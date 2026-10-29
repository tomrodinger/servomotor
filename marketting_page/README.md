# Servomotor marketing page

The current page is hand-authored in `final/index.html`. Start with
[`STATE_OF_PLAY.md`](STATE_OF_PLAY.md) and
[`MARKETING_PAGE_WORKFLOW.md`](MARKETING_PAGE_WORKFLOW.md) for the source register,
editing rules, preview pipeline and deployment details.

## Local documentation and preview checks

```sh
python3 check_facts.py
python3 deploy_to_store.py --dry-run
```

The dry run rebuilds `preview/` from the current page without copying into the
separate e-commerce working tree. Use `python3 serve.py` for the standalone
local page. Rebuild the slogan engine with `build_page_engine.py` only when
its effects or keep list change.

`python3 deploy_to_store.py` without `--dry-run` copies the preview into the
separate store working tree; it is an explicit deployment step and does not
commit or publish. Follow the workflow document before doing that.

## Content sources

- Command definitions: `../python_programs/servomotor/motor_commands.json`
- Firmware/API protocol: `../common_source_files/PROTOCOL_SPEC.md`
- Product specifications: `../servomotor_datasheets/specs.py`
- Datasheet version: `../servomotor_datasheets/versions.txt`
- Unit system: `../python_programs/servomotor/unit_conversions_M3.json`
- Page feature/protocol/company text: `features.txt`, `communication.txt`,
  `company_profile.txt` and `package.txt`

The page links to the command catalog rather than embedding a numbered command
list, so the PID getter is documented through that catalog and the API references.

## Archived generation path

`generate_webpage.py` and `templates/next_index_template.jsx` describe the old
stacked template. Its generation entry point is guarded because it can replace
the hand-authored design. Use the current dry-run/deploy pipeline above for page
updates. The generic `copy_to_ecommerce()` function in that module is still used
by `deploy_to_store.py`.
