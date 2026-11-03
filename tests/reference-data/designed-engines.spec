# Engines built by the designer from their makers' data sheets alone
# (`EngineLabDesignBench --spec`), the way a user builds one: cycle, use,
# layout, cylinders, bore, stroke, compression ratio and the rated power speed.
# Nothing else is set: no cam, runner, exhaust or combustion figure, and no
# value is ever tuned to land on the maker's figures. Sources and the rules of
# the set are in designed-manufacturer-sources.md; the points in
# designed-manufacturer-points.csv.
#
# The redline is set at the rated power speed / 0.9, the designer's own rule
# read backwards (peak power = 0.9 x redline), so that the designed engine is
# built for the speed its maker rates it at.
#
# Tuning set: decisions about the physics are taken on these.
key=cbr1000rrr; name=D-CBR1000RR-R; cycle=4t; use=sport_motorcycle; layout=inline; cylinders=4; bore=81.0; stroke=48.5; compression_ratio=13.0; redline_rpm=16100
key=cp4; name=D-CP4 MT-10; cycle=4t; use=sport_motorcycle; layout=inline; cylinders=4; bore=79.0; stroke=50.9; compression_ratio=12.0; redline_rpm=12800
key=hayabusa; name=D-Hayabusa 1999; cycle=4t; use=sport_motorcycle; layout=inline; cylinders=4; bore=81.0; stroke=63.0; compression_ratio=11.0; redline_rpm=10900
key=cp2; name=D-CP2 MT-07; cycle=4t; use=road_motorcycle; layout=inline; cylinders=2; bore=80.0; stroke=68.6; compression_ratio=11.5; redline_rpm=9800
key=cp3; name=D-CP3 MT-09; cycle=4t; use=road_motorcycle; layout=inline; cylinders=3; bore=78.0; stroke=62.1; compression_ratio=11.5; redline_rpm=11200
key=k20a; name=D-K20A Euro-R; cycle=4t; use=sport_car; layout=inline; cylinders=4; bore=86.0; stroke=86.0; compression_ratio=11.5; redline_rpm=8900
key=ls3; name=D-LS3; cycle=4t; use=sport_car; layout=v; cylinders=8; bank_angle=90; bore=103.25; stroke=92.0; compression_ratio=10.7; valves_per_side=1; redline_rpm=6600
key=m8_117; name=D-M8 117; cycle=4t; use=cruiser_motorcycle; layout=v; cylinders=2; bank_angle=45; bore=103.5; stroke=114.3; compression_ratio=10.2; valves_per_side=2; redline_rpm=5200
#
# Control set: never used to decide a change, only to check that it generalises.
key=ls7; name=D-LS7; cycle=4t; use=sport_car; layout=v; cylinders=8; bank_angle=90; bore=104.8; stroke=101.6; compression_ratio=11.0; valves_per_side=1; redline_rpm=7000
key=panigale1299; name=D-Panigale 1299; cycle=4t; use=sport_motorcycle; layout=v; cylinders=2; bank_angle=90; bore=116.0; stroke=60.8; compression_ratio=12.6; redline_rpm=11700
key=sc59; name=D-SC59 CBR1000RR 2008; cycle=4t; use=sport_motorcycle; layout=inline; cylinders=4; bore=76.0; stroke=55.1; compression_ratio=12.3; redline_rpm=13400
