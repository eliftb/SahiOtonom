#!/usr/bin/env python3
"""
parkur.world'den bir nesneyi isme gore KALDIRIR.
Hem trafik isiklarini (<model ...>) hem normal isiklari (<light ...>) siler.

Kullanim:
  python3 remove_traffic_light.py <isim>

Ornekler:
  python3 remove_traffic_light.py light_model_4     # trafik isigi (model)
  python3 remove_traffic_light.py my_light_1        # normal isik

Notlar:
  - <isim> tam eslesmeli.
  - Calistirmadan once otomatik yedek alir: parkur.world.bak
  - ground_plane, revize-parkur ve sun silinemez (kritik).
  - Farkli bir dunya: PARKUR_WORLD=/yol/dunya.world python3 remove_traffic_light.py ...
"""
import re
import sys
import os
import shutil

WORLD = os.environ.get(
    "PARKUR_WORLD",
    "/home/elifnur/Desktop/sahi_otonom/worlds/parkur.world",
)

PROTECTED = ("ground_plane", "revize-parkur", "sun")


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)

    name = sys.argv[1]

    if name in PROTECTED:
        print("HATA: '%s' kritik bir nesne, silinemez." % name)
        sys.exit(1)

    if not os.path.exists(WORLD):
        print("HATA: dunya dosyasi bulunamadi: %s" % WORLD)
        sys.exit(1)

    with open(WORLD) as f:
        s = f.read()

    model_tag = "<model name='%s'>" % name
    if model_tag in s:
        pos = s.index(model_tag)
        start = s.rindex("\n", 0, pos) + 1
        end = s.index("</model>", pos) + len("</model>")
        kind = "trafik isigi/model"
    else:
        m = re.search(r"<light\b[^>]*name=['\"]%s['\"][^>]*>" % re.escape(name), s)
        if not m:
            print("HATA: '%s' adinda bir model/isik bulunamadi." % name)
            sys.exit(1)
        start = s.rindex("\n", 0, m.start()) + 1
        end = s.index("</light>", m.start()) + len("</light>")
        kind = "isik"

    out = s[:start] + s[end:]
    out = re.sub(r"\n{3,}", "\n\n", out)

    shutil.copy(WORLD, WORLD + ".bak")
    with open(WORLD, "w") as f:
        f.write(out)

    print("OK: '%s' silindi (%s)." % (name, kind))
    print("Yedek: %s.bak" % WORLD)
    print("Gormek icin Gazebo'yu yeniden baslat:  ./launch_gazebo_parkur.sh")


if __name__ == "__main__":
    main()
