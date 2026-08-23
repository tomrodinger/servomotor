#!/usr/bin/env python3
"""Deploy the HAND-AUTHORED marketing page into the e-commerce store.

This REPLACES HALF of `generate_webpage.py`, and reuses the other half.

`generate_webpage.py main()` is two steps: `create_unified_preview()` builds `preview/` from
`templates/next_index_template.jsx`, then `copy_to_ecommerce()` copies `preview/` into the store.
Since 2026-08-22 the page is hand-authored in `final/index.html` (MARKETING_PAGE_WORKFLOW.md), so
the GENERATE step is obsolete — running it would rebuild the component from the old stacked template
and publish that over the new page. The COPY step is not obsolete at all: it is the deploy, and it
copies exactly the three paths this script writes into `preview/`.

So this script does the generate step from the hand-authored page and then calls
`generate_webpage.copy_to_ecommerce()` for the copy. One deployer, not two that drift apart.

How it ports HTML into a React component:
NOT by converting markup to JSX. That means rewriting every attribute, self-closing every tag and
re-escaping every entity across 150 KB of markup — a large surface for silent breakage. Instead the
body is injected with `dangerouslySetInnerHTML` and the CSS with a plain <style> tag, so the markup
the browser gets is byte-for-byte the markup that was reviewed. React never parses it, so it cannot
mangle it. Scripts inside injected HTML do NOT execute, so the slogan engine is lifted out and run
from a `useEffect` after mount.

FIVE adaptations are applied on the way out, because a standalone page and a page embedded in a
store are not the same document:

  1. CSS SCOPING. The page's stylesheet is global — it styles `nav`, `footer`, `a`, `img`, `table`
     and 22 other bare elements. Dropped into the store as-is it would restyle the store's own
     Header, SiteFooter and cart. Every selector is therefore rewritten under `.mkt-root`, the
     wrapper this component renders. `html`/`body`/`:root` map onto that wrapper; `.js`, which the
     reveal script puts on <html>, stays an ancestor. STORE_RESET handles the reverse leak for
     bare element selectors.

  2. COLLIDING CLASS NAMES ARE RENAMED. Scoping and STORE_RESET both miss the case where the store
     and the page use the SAME class name — globals.css styles `.btn`, `.btn-primary` and `.card`,
     and so does the page. See the comment at step 2b.

  3. THE PAGE'S OWN <nav> IS REMOVED, along with the script that recolours it on scroll. The store
     renders a global <Header/> in `pages/_app.js`; keeping both would stack two navigation bars.
     The nav is `position:sticky`, so removing it costs no layout compensation.

  4. ASSET PATHS. `../foo.png` becomes `/marketing/images/foo.png` and the file is copied.

  5. SAME-SITE LINKS BECOME ROOT-RELATIVE, so a deployment other than production (staging) can be
     clicked through. See step 4b.

The emitted component also tears the engine down on unmount — this is a single-page app, and
without that every navigation away from the homepage leaks a 60 fps rAF loop. See the effect.

Everything else — markup, copy, layout, the slogan engine — ships unchanged.

  python3 deploy_to_store.py            # build + copy into the store working tree
  python3 deploy_to_store.py --dry-run  # build into preview/ only, touch nothing else

It never commits and never pushes. Publishing to the live site is a separate, deliberate act.
"""
import argparse
import json
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(ROOT, "final", "index.html")
PREVIEW = os.path.join(ROOT, "preview")
STORE = os.path.abspath(os.path.join(ROOT, "..", "..", "AI_testing", "selling_web_site"))

# where a referenced asset actually lives, relative to marketting_page/
ASSET_DIRS = [ROOT, os.path.join(ROOT, "transparent")]

SCOPE = ".mkt-root"

# Scoping stops the PAGE's global rules escaping into the store. This stops the STORE's global
# rules leaking the other way. `styles/globals.css` styles bare `h1`-`h6`, `p`, `a` and `main`;
# where the page never declares that property itself (a heading's colour, say, which the standalone
# page simply inherits from <body>), the store's element rule wins and the result is wrong — the
# hero headline rendered #0F172A navy on the near-black hero.
#
# Every selector here is wrapped in :where(), which contributes NO specificity, so these rules sit
# at (0,1,0) — above the store's bare `h1` (0,0,1) and below every rule the page itself writes
# (`.mkt-root h1` is (0,1,1)). They restore the UA defaults the page was authored against; the
# page's own `* { margin:0; padding:0 }` comes later in the sheet and still wins on the box model.
STORE_RESET = """/* ---- injected by deploy_to_store.py -------------------------------------------------------
   Neutralise the store's global element styling inside the marketing block. Zero-specificity
   :where() selectors: they beat bare `h1 {}` from globals.css and lose to every page rule. */
.mkt-root :where(h1,h2,h3,h4,h5,h6){margin:0;font-weight:bold;color:inherit;
  line-height:inherit;letter-spacing:inherit;}
.mkt-root :where(h1){font-size:2em;}
.mkt-root :where(h2){font-size:1.5em;}
.mkt-root :where(h3){font-size:1.17em;}
.mkt-root :where(h4){font-size:1em;}
.mkt-root :where(h5){font-size:.83em;}
.mkt-root :where(h6){font-size:.67em;text-transform:none;}
.mkt-root :where(p){margin:0;color:inherit;line-height:inherit;}
.mkt-root :where(a){color:inherit;text-decoration:none;transition:none;}
.mkt-root :where(main){padding:0;min-height:auto;display:block;background:none;}
/* -------------------------------------------------------------------------------------- */
"""

# Selector heads that name an element ABOVE the wrapper. They must stay outside the scope class,
# with the scope inserted after them, or the rule stops matching.
ANCESTORS = ("html", ".js", "html.js")

# Page class names that also exist in the store's globals.css, mapped to a prefixed name.
# Filled in by collide_rename(); see the comment there for why renaming beats another reset layer.
RENAME = {}


def store_global_classes():
    """Class names the store's own globals.css styles."""
    p = os.path.join(STORE, "styles", "globals.css")
    if not os.path.exists(p):
        return set()
    css = re.sub(r"/\*.*?\*/", " ", open(p, encoding="utf-8").read(), flags=re.S)
    return set(re.findall(r"\.([A-Za-z_][\w-]*)", re.sub(r"\{[^{}]*\}", " ", css)))


def apply_rename(sel):
    """Rewrite renamed class tokens inside one selector."""
    for old_name, new_name in RENAME.items():
        sel = re.sub(r"\.%s(?![-\w])" % re.escape(old_name), "." + new_name, sel)
    return sel


def rename_in_markup(html):
    """Rewrite renamed class tokens, and ONLY inside class="..." attributes."""
    def one(m):
        toks = [RENAME.get(t, t) for t in m.group(1).split()]
        return 'class="%s"' % " ".join(toks)
    return re.sub(r'class="([^"]*)"', one, html)


# --------------------------------------------------------------------------------------------
# CSS scoping
# --------------------------------------------------------------------------------------------

def split_selectors(sel):
    """Split a selector list on top-level commas (not the ones inside :not(...) / :is(...))."""
    out, depth, buf = [], 0, ""
    for ch in sel:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(buf)
            buf = ""
        else:
            buf += ch
    out.append(buf)
    return [s.strip() for s in out if s.strip()]


def scope_one(sel):
    """Rewrite a single selector so it only matches inside the wrapper."""
    sel = apply_rename(sel)
    if sel in (":root", "html", "body"):
        return SCOPE
    if sel == "*":
        return "%s, %s *" % (SCOPE, SCOPE)
    m = re.match(r"^([^\s>+~]+)(.*)$", sel, re.S)
    head, rest = m.group(1), m.group(2)
    if head == "body":
        return SCOPE + rest
    if head in ANCESTORS:
        return "%s %s%s" % (head, SCOPE, rest)
    return "%s %s" % (SCOPE, sel)


def scope_css(css):
    """Prefix every rule in `css` with the wrapper class.

    Block-aware: recurses into @media/@supports/@layer/@container, and leaves @keyframes,
    @font-face and @page bodies alone (their contents are not selectors).
    """
    out, i, n, buf = [], 0, len(css), ""
    while i < n:
        ch = css[i]
        if ch == "{":
            depth, j = 1, i + 1
            while j < n and depth:
                if css[j] == "{":
                    depth += 1
                elif css[j] == "}":
                    depth -= 1
                j += 1
            prelude, body = buf, css[i + 1:j - 1]
            buf, i = "", j
            # Peel leading whitespace and comments off first, and keep them: a comment sitting
            # above a rule is part of the prelude, and an @media introduced by one must still be
            # recognised as an at-rule rather than scoped like a selector.
            cm = re.match(r"^(\s*(?:/\*.*?\*/\s*)*)(.*)$", prelude, re.S)
            lead, head = (cm.group(1), cm.group(2).strip()) if cm else ("", prelude.strip())
            if head.startswith("@"):
                at = head.split()[0].split("(")[0].lower()
                if at in ("@media", "@supports", "@layer", "@container", "@scope"):
                    out.append("%s%s{%s}" % (lead, head, scope_css(body)))
                else:                                   # @keyframes, @font-face, @page
                    out.append("%s%s{%s}" % (lead, head, body))
            else:
                sels = ", ".join(scope_one(s) for s in split_selectors(head))
                out.append("%s%s{%s}" % (lead, sels, body))
        else:
            buf += ch
            i += 1
    out.append(buf)
    return "".join(out)


def check_scoped(original, scoped):
    """Cheap sanity net: same number of declaration blocks, balanced braces, nothing unscoped."""
    problems = []
    if original.count("{") != scoped.count("{"):
        problems.append("block count changed: %d -> %d" % (original.count("{"), scoped.count("{")))
    if scoped.count("{") != scoped.count("}"):
        problems.append("unbalanced braces in the scoped CSS")
    leaked = []
    for m in re.finditer(r"(^|[}])([^{}@]+)\{", scoped):
        for s in split_selectors(re.sub(r"/\*.*?\*/", " ", m.group(2), flags=re.S)):
            if s and SCOPE not in s and not s.startswith(("html", ".js", "0%", "100%", "from", "to")):
                leaked.append(s)
    if leaked:
        problems.append("%d selector(s) escaped the scope, e.g. %s"
                        % (len(leaked), "; ".join(sorted(set(leaked))[:5])))
    return problems


# --------------------------------------------------------------------------------------------

def find_asset(ref):
    """'../transparent/x.png' -> the real file on disk."""
    name = ref.split("/")[-1]
    cand = os.path.normpath(os.path.join(ROOT, ref.lstrip("./")))
    if os.path.exists(cand):
        return cand
    for d in ASSET_DIRS:
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    return None


def check_preview_is_complete():
    """`copy_to_ecommerce()` rmtree's public/marketing/ before copying, so preview/ must carry
    EVERYTHING the store should end up with — not just what this build produced.

    The page itself only references files that land in `images/`. `logos/Gearotons_Logo.png` is used
    by other parts of the store, so if preview/ ever loses it, the copy would silently delete it
    from the store. Fail loudly instead.
    """
    logo = os.path.join(PREVIEW, "public", "marketing", "logos", "Gearotons_Logo.png")
    if os.path.exists(logo):
        return []
    live = os.path.join(STORE, "public", "marketing", "logos", "Gearotons_Logo.png")
    if os.path.exists(live):                       # rescue it so the round-trip stays lossless
        os.makedirs(os.path.dirname(logo), exist_ok=True)
        shutil.copy2(live, logo)
        return []
    return ["public/marketing/logos/Gearotons_Logo.png is in neither preview/ nor the store; "
            "copy_to_ecommerce() would leave the store without it"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true",
                    help="build into preview/ but do not touch the store")
    ap.add_argument("--keep-nav", action="store_true",
                    help="do NOT strip the page's own <nav> (only if the store stops rendering one)")
    a = ap.parse_args()

    if not os.path.exists(PAGE):
        print("no page at %s" % PAGE)
        return 2
    src = open(PAGE, encoding="utf-8").read()

    # ---- 1. split the page -------------------------------------------------------------
    style = "\n".join(re.findall(r"<style[^>]*>(.*?)</style>", src, re.S))
    scripts = re.findall(r"<script[^>]*>(.*?)</script>", src, re.S)
    body_m = re.search(r"<body[^>]*>(.*)</body>", src, re.S)
    if not body_m:
        print("could not find <body>")
        return 2
    body = body_m.group(1)
    body = re.sub(r"<script[^>]*>.*?</script>", "", body, flags=re.S)   # scripts move to useEffect
    body_class = (re.search(r"<body([^>]*)>", src).group(1) or "")
    body_class = (re.search(r'class="([^"]*)"', body_class) or [None, ""])[1]

    notes = []

    # ---- 2. store adaptation: the store owns the navigation ----------------------------
    if not a.keep_nav:
        body, n_nav = re.subn(r"[ \t]*<nav\b.*?</nav>\s*", "", body, flags=re.S)
        if n_nav != 1:
            print("!! expected exactly one <nav> to strip, removed %d — check final/index.html"
                  % n_nav)
            return 2
        before = len(scripts)
        scripts = [s for s in scripts if "querySelector('nav')" not in s
                   and 'querySelector("nav")' not in s]
        notes.append("stripped the page's own <nav> and %d nav script(s); the store renders its own "
                     "<Header/>" % (before - len(scripts)))

    # ---- 2b. rename class names that collide with the store's globals.css ---------------
    # Scoping keeps the PAGE's rules off the store. STORE_RESET keeps the store's bare ELEMENT
    # rules off the page. Neither covers the third case: the store's globals.css also styles
    # utility CLASSES (.btn, .btn-primary, .card) and the page happens to use those same names.
    # Scoping wins on the properties the page declares, but every property the store declares and
    # the page does NOT is silently inherited from the store — verified live: the spec cards
    # picked up a white background, a drop shadow, `overflow:hidden` and a hover shadow, and the
    # hero CTA picked up a hover nudge. Nobody chose any of that.
    #
    # Renaming rather than adding another reset layer, because a reset has to enumerate every
    # property the store might ever set, and gets it wrong the moment globals.css grows a line.
    # A name the store does not style cannot be reached at all.
    used = set()
    for m in re.finditer(r'class="([^"]*)"', body):
        used.update(m.group(1).split())
    collisions = sorted(used & store_global_classes())
    if collisions:
        # Renaming touches markup and CSS but NOT the lifted JS, so a class the script selects by
        # name would silently stop matching. Refuse rather than half-rename.
        js_all = "\n".join(scripts)
        risky = [c for c in collisions
                 if re.search(r"""['"`]\.?%s(?![-\w])""" % re.escape(c), js_all)]
        if risky:
            print("!! %s collide with the store's globals.css but are referenced in the page's "
                  "JS as strings — rename by hand, or the script will stop matching them: %s"
                  % (len(risky), risky))
            return 2
        RENAME.update({c: "mkt-" + c for c in collisions})
        body = rename_in_markup(body)
        notes.append("renamed %d class(es) that collide with the store's globals.css: %s"
                     % (len(collisions), ", ".join("%s->mkt-%s" % (c, c) for c in collisions)))

    # ---- 3. scope the stylesheet -------------------------------------------------------
    scoped = scope_css(style)
    css_problems = check_scoped(style, scoped)
    scoped = STORE_RESET + scoped          # reset FIRST: the page's own rules must come after
    notes.append("scoped %d CSS blocks under %s, prepended the store-globals reset"
                 % (style.count("{"), SCOPE))

    # ---- 4. rewrite asset paths and collect the files ----------------------------------
    refs = sorted(set(re.findall(r'(?:src|href)="(\.\./[^"]+\.(?:png|jpg|jpeg|svg|webp))"', src)))
    images_out = os.path.join(PREVIEW, "public", "marketing", "images")
    if os.path.isdir(images_out):
        shutil.rmtree(images_out)          # preview/ is ours to rebuild; the store's is not
    os.makedirs(images_out, exist_ok=True)
    copied, missing = [], []
    for ref in refs:
        p = find_asset(ref)
        if not p:
            missing.append(ref)
            continue
        shutil.copy2(p, os.path.join(images_out, os.path.basename(p)))
        copied.append(os.path.basename(p))
        web = "/marketing/images/" + os.path.basename(p)
        body = body.replace(ref, web)
        scoped = scoped.replace(ref, web)

    # ---- 4b. make same-site links root-relative -----------------------------------------
    # The standalone page is written with absolute https://gearotons.com/... URLs because it
    # can be published on its own. Inside the store those are wrong: they pin the visitor to
    # PRODUCTION, so on Amplify staging the primary CTA jumped to a different deployment
    # running a different build, and the staging copy could not be reviewed by clicking
    # through it. Root-relative paths work on every deployment and additionally let Next
    # route client-side instead of forcing a full page load.
    # Only the apex is rewritten — tutorial.gearotons.com is genuinely another origin.
    site_links = len(re.findall(r'https://(?:www\.)?gearotons\.com', body))
    body = re.sub(r'https://(?:www\.)?gearotons\.com/store', '/store', body)
    body = re.sub(r'https://(?:www\.)?gearotons\.com(?=["\'\s>])', '/', body)
    left = len(re.findall(r'https://(?:www\.)?gearotons\.com', body))
    notes.append("made %d same-site link(s) root-relative (%d left)" % (site_links - left, left))

    # ---- 5. emit the component ---------------------------------------------------------
    js = "\n\n".join(s.strip() for s in scripts if s.strip())
    component = COMPONENT_TEMPLATE.replace("__BODY__", json.dumps(body)) \
                                  .replace("__STYLE__", json.dumps(scoped)) \
                                  .replace("__SCRIPT__", json.dumps(js)) \
                                  .replace("__BODYCLASS__", json.dumps(body_class)) \
                                  .replace("__SCOPE__", json.dumps(SCOPE.lstrip(".")))
    os.makedirs(os.path.join(PREVIEW, "components"), exist_ok=True)
    open(os.path.join(PREVIEW, "components", "MarketingContent.js"), "w",
         encoding="utf-8").write(component)

    left = re.findall(r"__[A-Z]+__", component)
    print("built from final/index.html")
    print("  body      %7d bytes" % len(body))
    print("  css       %7d bytes  (%d -> %d after scoping)" % (len(scoped), len(style), len(scoped)))
    print("  script    %7d bytes  (%d blocks -> useEffect)" % (len(js), len(scripts)))
    print("  assets    %7d copied" % len(copied))
    for n in notes:
        print("  - %s" % n)
    if left:
        print("  !! unreplaced template tokens: %s" % sorted(set(left)))
        return 2
    if css_problems:
        print("  !! CSS scoping problems:")
        for p in css_problems:
            print("       %s" % p)
        return 2
    if missing:
        print("  !! MISSING assets, the page will 404 on these:")
        for m in missing:
            print("       %s" % m)
        return 2

    if a.dry_run:
        print("\ndry run: preview/ updated, store untouched")
        return 0

    # ---- 6. hand off to the EXISTING deployer -------------------------------------------
    # The copy step is not ours to reinvent: `generate_webpage.copy_to_ecommerce()` is the deploy
    # Tom has always used, and it already copies exactly the three paths this build writes into
    # preview/. Only the GENERATE half of generate_webpage.py is obsolete (it rebuilds the component
    # from templates/next_index_template.jsx, the old stacked design). So we replace the generate
    # half and call straight into the copy half — one deployer, not two that can drift.
    if not os.path.isdir(STORE):
        print("\nstore not found at %s" % STORE)
        return 2

    gaps = check_preview_is_complete()
    if gaps:
        print("\n!! refusing to copy — preview/ is missing files the store needs:")
        for g in gaps:
            print("     %s" % g)
        return 2

    # `copy_to_ecommerce` skips a source it cannot find, which is how the store's own
    # styles/Marketing.module.css survives: the new component ships its CSS inline and does not
    # import the module, so we deliberately do not stage one.
    stale_css = os.path.join(PREVIEW, "styles", "Marketing.module.css")
    if os.path.exists(stale_css):
        os.remove(stale_css)

    dst = os.path.join(STORE, "components", "MarketingContent.js")
    if os.path.exists(dst) and not os.path.exists(dst + ".bak"):
        shutil.copy2(dst, dst + ".bak")     # the outgoing one had uncommitted edits
        print("\nkept the previous component as components/MarketingContent.js.bak")

    from generate_webpage import copy_to_ecommerce
    copy_to_ecommerce(PREVIEW, STORE)

    print("\nCopied into the store WORKING TREE only. Nothing committed, nothing pushed.")
    print("Review it, then commit and push yourself if you want it live.")
    return 0


COMPONENT_TEMPLATE = '''import { useEffect, useRef } from 'react';

/**
 * GENERATED by marketting_page/deploy_to_store.py from the hand-authored final/index.html.
 * DO NOT HAND-EDIT — edit final/index.html and redeploy, or the next deploy overwrites you.
 *
 * The markup is injected rather than converted to JSX so that what ships is byte-for-byte what was
 * reviewed in the browser. Injected <script> tags never execute, so the slogan engine runs from the
 * effect below after mount.
 *
 * Two nested wrappers, on purpose:
 *   .mkt-root      every CSS rule from the page is scoped under this, so the page's global
 *                  `nav {}` / `footer {}` / `a {}` rules cannot reach the store's own Header,
 *                  SiteFooter or cart. It also stands in for `html`/`body`/`:root`.
 *   BODY_CLASS     what was on <body> in the standalone page (e.g. "hero-dark"). It has to be a
 *                  DESCENDANT of .mkt-root for scoped rules like `.hero-dark .hero-final` to match.
 *
 * The page's own <nav> is removed at build time; the store renders a global <Header/> in _app.js.
 */
export default function MarketingContent() {
  const ranRef = useRef(false);

  useEffect(() => {
    if (ranRef.current) return undefined;   // guard a double invoke with no cleanup between
    ranRef.current = true;
    // Left deliberately: `window.__marketingEffect` is the only way to tell "the effect never ran"
    // (a hydration failure) apart from "the engine threw" from outside the page.
    window.__marketingEffect = { ran: true, len: (MARKETING_SCRIPT || '').length };

    // The lifted script starts a self-re-arming requestAnimationFrame loop and registers
    // listeners on window and document. This is a single-page app: React unmounts this component
    // on every client-side navigation away from the homepage, and without a teardown the loop and
    // the listeners outlive it. Home -> Store -> Home three times leaves four players animating
    // detached DOM at 60 fps, and only the newest is reachable to stop. So record what the script
    // starts and undo exactly that. Listeners on elements INSIDE the injected markup need no
    // cleanup — React removes those nodes and the listeners go with them.
    const liveRaf = new Set();
    const nativeRaf = window.requestAnimationFrame;
    const nativeCancel = window.cancelAnimationFrame;
    window.requestAnimationFrame = function (cb) {
      const id = nativeRaf.call(window, function (t) { liveRaf.delete(id); return cb(t); });
      liveRaf.add(id);
      return id;
    };
    window.cancelAnimationFrame = function (id) {
      liveRaf.delete(id);
      return nativeCancel.call(window, id);
    };

    const targets = [window, document];
    const added = [];
    const prevAdd = new Map();
    targets.forEach((t) => {
      prevAdd.set(t, Object.prototype.hasOwnProperty.call(t, 'addEventListener')
        ? t.addEventListener : null);
      const inherited = t.addEventListener;
      t.addEventListener = function () {
        added.push([t, arguments]);
        return inherited.apply(t, arguments);
      };
    });

    try {
      // eslint-disable-next-line no-new-func
      new Function(MARKETING_SCRIPT)();
      window.__marketingEffect.ok = true;
    } catch (err) {
      window.__marketingEffect.error = String((err && err.message) || err);
      console.error('[marketing] slogan engine failed to start:', err);
    }

    return () => {
      try {
        if (window.__slogan && typeof window.__slogan.pause === 'function') {
          window.__slogan.pause();
        }
      } catch (e) { /* the engine is going away regardless */ }
      added.forEach(([t, args]) => {
        try { t.removeEventListener.apply(t, args); } catch (e) { /* ignore */ }
      });
      targets.forEach((t) => {
        const prev = prevAdd.get(t);
        if (prev) { t.addEventListener = prev; } else { delete t.addEventListener; }
      });
      liveRaf.forEach((id) => { try { nativeCancel.call(window, id); } catch (e) { /* ignore */ } });
      liveRaf.clear();
      window.requestAnimationFrame = nativeRaf;
      window.cancelAnimationFrame = nativeCancel;
      window.__marketingEffect.cleanedUp = true;
      ranRef.current = false;      // a remount is allowed to start it again
    };
  }, []);

  return (
    <div className={SCOPE_CLASS}>
      <style dangerouslySetInnerHTML={{ __html: MARKETING_CSS }} />
      <div className={BODY_CLASS} dangerouslySetInnerHTML={{ __html: MARKETING_HTML }} />
    </div>
  );
}

const SCOPE_CLASS = __SCOPE__;
const BODY_CLASS = __BODYCLASS__;
const MARKETING_CSS = __STYLE__;
const MARKETING_HTML = __BODY__;
const MARKETING_SCRIPT = __SCRIPT__;
'''

if __name__ == "__main__":
    sys.exit(main())
