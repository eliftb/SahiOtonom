#!/usr/bin/env python3
"""
parkur.world'e KALICI normal isik (aydinlatma) ekler.
(Bu trafik isigi DEGIL; trafik isigi icin add_traffic_light.py kullan.)

Kullanim:
  python3 add_light.py <isim> <x> <y> <z> [tip] [range]

  <isim>  : benzersiz isim (orn. my_light_1). Istedigin ismi verebilirsin.
  <x y z> : konum, metre.
  [tip]   : point (varsayilan) | spot | directional
  [range] : isigin etki menzili (metre, varsayilan 20). point/spot icin gecerli.

Ornekler:
  python3 add_light.py my_light_1 14.544 -37.91 3.0
  python3 add_light.py my_light_2 10 5 4 point 30

Notlar:
  - point isik her yone aydinlatir (yonelim onemsiz).
  - Calistirmadan once otomatik yedek alir: parkur.world.bak
  - Farkli bir dunya: PARKUR_WORLD=/yol/dunya.world python3 add_light.py ...
"""
import sys
import os
import shutil

WORLD = os.environ.get(
    "PARKUR_WORLD",
    "/home/elifnur/Desktop/sahi_otonom/worlds/parkur.world",
)

TEMPLATE = """    <light type="{ltype}" name="{name}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>{rng}</range>
        <constant>0.3</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>0 0 -1</direction>
      <cast_shadows>0</cast_shadows>
    </light>
"""


def main():
    a = sys.argv[1:]
    if len(a) < 4 or len(a) > 6:
        print(__doc__)
        sys.exit(1)

    name, x, y, z = a[0], a[1], a[2], a[3]
    ltype = a[4] if len(a) >= 5 else "point"
    rng = a[5] if len(a) >= 6 else "20"

    if ltype not in ("point", "spot", "directional"):
        print("HATA: tip 'point', 'spot' veya 'directional' olmali.")
        sys.exit(1)

    if not os.path.exists(WORLD):
        print("HATA: dunya dosyasi bulunamadi: %s" % WORLD)
        sys.exit(1)

    with open(WORLD) as f:
        s = f.read()

    if ('name="%s"' % name) in s or ("name='%s'" % name) in s:
        print("HATA: '%s' adinda bir model/isik zaten var. Farkli isim sec." % name)
        sys.exit(1)

    block = TEMPLATE.format(ltype=ltype, name=name, x=x, y=y, z=z, rng=rng)

    if "\n    <state " in s:
        idx = s.index("\n    <state ")
    else:
        idx = s.rindex("\n", 0, s.rindex("</world>"))

    out = s[:idx] + "\n\n" + block + s[idx:]

    shutil.copy(WORLD, WORLD + ".bak")
    with open(WORLD, "w") as f:
        f.write(out)

    print("OK: '%s' (%s isik) eklendi  konum: %s %s %s" % (name, ltype, x, y, z))
    print("Yedek: %s.bak" % WORLD)
    print("Gormek icin Gazebo'yu yeniden baslat:  ./launch_gazebo_parkur.sh")


if __name__ == "__main__":
    main()
