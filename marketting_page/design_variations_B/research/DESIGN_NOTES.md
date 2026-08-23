# Design language notes — pass 2 (structural rebuild)

**Pass 2 goal:** Each of v1–v40 has a **genuinely different page architecture**
(DOM structure / layout system), not merely different colors/fonts/buttons on a
shared skeleton. Pass 1 was rejected for visual-only variation.

Research method: market-cap ranking from CompaniesMarketCap (2026-07-17);
design notes from established public brand systems and marketing-site layout
patterns. No company trademarks, logos, product names, or copy — design
language only. All content is Gearotons M17.

## Pass 2 structural approach

| # | Label | Architecture (what differs) |
|---|-------|------------------------------|
| v1 | Precision Lab | Asymmetric 50/50 hero; sticky side rail + 2-col feature grid |
| v2 | Dark Command | Full-viewport dark stage; **horizontal scroll** capability strip |
| v3 | Editorial Serif | Magazine masthead; multi-column newspaper text; full-bleed photo break; pull quote |
| v4 | Soft Consumer | **Bento/masonry** CSS grid homepage of mixed-span cards |
| v5 | Blueprint Grid | Blueprint grid bg; two-pane drawing + numbered callouts; dimension sheets |
| v6 | Bold Poster | Huge stacked display type; thick black borders; 3-col model band |
| v7 | Swiss Grid | 12-col modular grid; numbered modules; red accent; Helvetica |
| v8 | Warm Workshop | Irregular **polaroid photo wall** + kraft paper notes |
| v9 | Neon Arcade | **Split dual panels**; game-UI stat bars; neon HUD |
| v10 | Luxury Minimal | Fixed tiny nav; **sequential single-column** scroll chapters; huge whitespace |
| v11 | NVIDIA-style | Stacked full-width story heroes; **horizontal card rails** |
| v12 | Apple-style | Centered full-bleed product units stacked; promo **2-up** grid; pill links |
| v13 | Google-style | Material multi-card auto-fill grid; soft elevation cards |
| v14 | Microsoft-style | **Mega product nav strip**; large hero; secondary **2×2 promo** cards |
| v15 | Amazon-style | Dark nav + fake search; **left gallery + right buy-box** PDP; thumb JS |
| v16 | TSMC-style | Institutional hero; **4 technology pillars** row; corporate table |
| v17 | Broadcom-style | Dark enterprise; **model tabs with JS**; dense datasheet blocks |
| v18 | SpaceX-style | **Sequential full-viewport** black sections; outlined CTAs; mission stats |
| v19 | Aramco-style | **Left sticky side navigation** + report-style single main column |
| v20 | Meta-style | Soft cards; **connection network diagram** (host + motor nodes) |
| v21 | Tesla-style | **100vh cinematic hero**; dual order CTAs; sticky model selector bar; filmstrip |
| v22 | Samsung-style | OLED product stage; **vertical numbered feature list** (big numerals) |
| v23 | Berkshire-style | Times longform **annual letter**; plain HTML; single narrow column |
| v24 | Eli Lilly-style | Medical calm; **horizontal 4-step pathway** process strip |
| v25 | Micron-style | Dark tech; **top metrics ribbon**; dense datasheet |
| v26 | Walmart-style | Retail blue/yellow; **category aisle grid** of applications |
| v27 | JPMorgan-style | Navy; **KPI strip**; multi-column report + sticky side cards |
| v28 | SK Hynix-style | **3 family comparison cards** side-by-side with featured middle |
| v29 | AMD-style | Aggressive stack; **big torque number product cards** stacked |
| v30 | Visa-style | Deep blue/gold; **horizontal 4-step how-it-works** overlapping hero |
| v31 | ASML-style | Near-black; light weights; deep **3-col system architecture** + 2-pane data |
| v32 | ExxonMobil-style | Industrial; **alternating half-image / half-text** application splits |
| v33 | J&J-style | Warm red; **storytelling bands** alternating image/text |
| v34 | Tencent-style | Dense **multi-panel app grid** over colorful hero |
| v35 | Intel-style | Blue gradient; **vertical layered architecture stack** diagram |
| v36 | Mastercard-style | Black modern; **large rounded 2×2 cards** + stacked content cards |
| v37 | AbbVie-style | Navy/orange; **clinical data table emphasis** as primary section |
| v38 | Applied Materials-style | Dark teal; **5-step process flow** industrial equipment |
| v39 | Cisco-style | **Network node diagram** (motors as bus nodes) as hero structure |
| v40 | Bank of America-style | Blue/red; **three product tier columns** banking comparison |

## Per-company notes (round 2)

### v11 NVIDIA
Black canvas, neon lime (#76b900), stacked story heroes, horizontal product rails.

### v12 Apple
SF-like system stack, huge centered product units, pill blue links (#0071e3), promo 2-up.

### v13 Alphabet / Google
Material white, colorful dots, soft elevation cards, friendly weight 400 headlines.

### v14 Microsoft
Mega nav strip, Fluent blue (#0067b8), left-accent promo cards 2×2.

### v15 Amazon
Navy nav (#131921), orange CTA, left thumbs + main image, right buy-box PDP density.

### v16 TSMC
Corporate navy/red, multi-pillar technology sections, institutional tables.

### v17 Broadcom
Black enterprise header, crimson accent, **tabbed product UI** (JS), dense datasheet.

### v18 SpaceX
Full-viewport black sections, huge uppercase, outlined white buttons, mission brief.

### v19 Saudi Aramco
Deep blue + cyan, **left sticky side nav**, report-style main column.

### v20 Meta
Meta blue (#0668E1), 24px rounded cards, connection network diagram.

### v21 Tesla
100vh cinematic hero, dual order-style CTAs, horizontal model selector, feature filmstrip.

### v22 Samsung
OLED black, Samsung blue, vertical feature list with large numbers.

### v23 Berkshire Hathaway
Times serif annual-letter longform, plain tables, minimal chrome.

### v24 Eli Lilly
Medical red, serif, horizontal clinical pathway steps.

### v25 Micron
Dark blue tech, cyan metrics ribbon, dense datasheet.

### v26 Walmart
Blue/yellow retail, aisle category grid, value-forward hero.

### v27 JPMorgan Chase
Navy finance, KPI strip (real specs only), multi-column report.

### v28 SK Hynix
Black/red, three family comparison cards side by side.

### v29 AMD
Black/red aggressive, big torque numbers on stack cards.

### v30 Visa
Deep blue + gold, 4-step how-it-works process.

### v31 ASML
Near-black precision, light font weights, deep technical sections.

### v32 ExxonMobil
Black/red industrial, split half-image half-text bands.

### v33 Johnson & Johnson
Warm red care, alternating image/text storytelling bands.

### v34 Tencent
Light dense multi-panel product grid, cyan accent.

### v35 Intel
Blue layered vertical architecture stack of integrated components.

### v36 Mastercard
Black modern, large rounded cards, bold simple sections.

### v37 AbbVie
Navy + orange, clinical-style data table as hero content.

### v38 Applied Materials
Dark teal industrial, 5-step process flow.

### v39 Cisco
Network node diagram (motors as nodes on bus), Cisco blue (#049fd9).

### v40 Bank of America
Flag blue/red, three product tier columns banking-style comparison.

## Approximations
- No headless screenshots in this run; structures derived from known marketing-site patterns.
- No trademarks/logos/product names of pastiche companies appear in any HTML.
