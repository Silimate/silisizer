# Silisizer
Operator resize for meeting timing.

Run the timing optimizer from Tcl after loading and linking the design:

```tcl
sta::silisize workdir
```

Pass `-all` to upsize every eligible offender found during each timing pass
instead of using adaptive batch sizing:

```tcl
sta::silisize -all workdir
```

Pass `-wns` to stop after three consecutive timing passes without WNS
improvement, even if other violating paths could still improve:

```tcl
sta::silisize -wns workdir
```

The policies can be combined:

```tcl
sta::silisize -all -wns workdir
```

`silisize` always creates `workdir/data/resized_cells.tsv` (header only when no
cells are resized) so Preqorsor can back-annotate SPEED==2 runs. Failure to
create that file is a hard error.
