#!/usr/bin/env python3
"""
parkur.world dosyasina KALICI trafik isigi ekler.

Kullanim:
  python3 add_traffic_light.py <isim> <x> <y> <z> <roll> <pitch> <yaw>

Ornek:
  python3 add_traffic_light.py light_model_5  -26.75 -0.64 2.27  3.1416 1.5516 3.1416

Notlar:
  - <isim> benzersiz olmali (ayni isimde model varsa hata verir).
  - x y z  -> metre cinsinden konum.
  - roll pitch yaw -> radyan cinsinden yonelim.
    En kolayi: Gazebo'da isigi elle yerlestir, sol paneldeki "pose"
    degerlerini (x y z roll pitch yaw) buraya yaz.
  - Dosyadaki ilk 'light_model' modelini sablon alir; yani kure lambalar
    ve one kaydirilmis (z=0.7) ayni stil otomatik kopyalanir.
  - Calistirmadan once otomatik yedek alir: parkur.world.bak
  - Farkli bir dunya icin: PARKUR_WORLD=/yol/dunya.world python3 add_traffic_light.py ...
"""
import re
import sys
import os
import shutil

WORLD = os.environ.get(
    "PARKUR_WORLD",
    "/home/elifnur/Desktop/sahi_otonom/worlds/parkur.world",
)


def main():
    if len(sys.argv) != 8:
        print(__doc__)
        sys.exit(1)

    name = sys.argv[1]
    pose = " ".join(sys.argv[2:8])

    if not os.path.exists(WORLD):
        print("HATA: dunya dosyasi bulunamadi: %s" % WORLD)
        sys.exit(1)

    with open(WORLD) as f:
        s = f.read()

    if ("<model name='%s'>" % name) in s or ('<model name="%s">' % name) in s:
        print("HATA: '%s' adinda bir model zaten var. Farkli bir isim sec." % name)
        sys.exit(1)

    # Sablon: dosyadaki ilk 'light_model' modeli (kure + z=0.7 hali)
    try:
        anchor = s.index("<model name='light_model'>")
    except ValueError:
        print("HATA: sablon olarak 'light_model' bulunamadi. Once en az bir isik olmali.")
        sys.exit(1)

    start = s.rindex("\n", 0, anchor) + 1            # satir basina (girintiyle) git
    end = s.index("</model>", anchor) + len("</model>")
    block = s[start:end]

    new_block = block.replace("<model name='light_model'>", "<model name='%s'>" % name, 1)
    # Sadece ilk <pose> (model pozu) degisir; link pozlarina dokunmaz.
    new_block = re.sub(r"<pose>[^<]*</pose>", "<pose>%s</pose>" % pose, new_block, count=1)

    # <state ...> blogundan hemen once ekle (yoksa </world> oncesi)
    if "\n    <state " in s:
        idx = s.index("\n    <state ")
    else:
        idx = s.rindex("</world>")
        idx = s.rindex("\n", 0, idx)

    out = s[:idx] + "\n\n" + new_block + s[idx:]

    shutil.copy(WORLD, WORLD + ".bak")
    with open(WORLD, "w") as f:
        f.write(out)

    print("OK: '%s' eklendi  (pose: %s)" % (name, pose))
    print("Yedek: %s.bak" % WORLD)
    print("Gormek icin Gazebo'yu yeniden baslat:  ./launch_gazebo_parkur.sh")


if __name__ == "__main__":
    main()
