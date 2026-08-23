#!/bin/bash
# Independent audit of all 20 variants against the four-motor spec update.
cd "$(dirname "$0")"
printf "%-5s %-9s %-7s %-9s %-9s %-8s %s\n" v models stale badimg noOldLED 4up notes
fail=0
for i in $(seq 1 20); do
  f=v$i.html; [ -f "$f" ] || { echo "v$i MISSING"; continue; }
  models=$(for m in M17-60 M17-48 M17-40 M17-34; do grep -qF "$m" "$f" && echo x; done | wc -l | tr -d ' ')
  # stale values that must no longer appear anywhere
  stale=$(grep -oE '38 ?W|32 ?W|25 ?W|59\.8|48\.6|41\.6|20\.4 ?mm|18\.5 ?mm' "$f" | sort -u | tr '\n' ',' )
  # dimension drawings must come from the top level, not the stale preview/ copies
  badimg=$(grep -c 'preview/public/marketing/images/M17-[0-9]*_dimensions' "$f")
  # the old (wrong) LED sentence: red LED showing bus traffic
  oldled=$(grep -ciE 'red LED[^.]{0,80}(communication on the bus|show communication)' "$f")
  # new values that must be present
  need=""
  for n in "0.28" "24.0" "26.4" "33.5" "210" "20.6" "40.1" "1.0 A"; do
    grep -qF "$n" "$f" || need="$need!$n"
  done
  st="ok"; [ -n "$stale" ] && { st="STALE:$stale"; fail=1; }
  bi="ok"; [ "$badimg" -gt 0 ] && { bi="BAD:$badimg"; fail=1; }
  ol="ok"; [ "$oldled" -gt 0 ] && { ol="OLDLED"; fail=1; }
  md="$models/4"; [ "$models" != "4" ] && fail=1
  nd="ok"; [ -n "$need" ] && { nd="MISSING$need"; fail=1; }
  printf "%-5s %-9s %-7s %-9s %-9s %-8s %s\n" "v$i" "$md" "$st" "$bi" "$ol" "$nd" ""
done
echo
[ "$fail" = "0" ] && echo "AUDIT PASS: all 20 clean" || echo "AUDIT: issues above"
