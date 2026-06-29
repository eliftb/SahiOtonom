# Trafik Işığı Ekleme Rehberi

Trafik ışıkları `parkur.world` dosyasının içinde yaşar. Her ışık bir `<model>` bloğudur
ve bir **pose** (konum + yönelim) ile tanımlanır. Dünya dosyasına yazılan her ışık,
Gazebo her açıldığında otomatik gelir. Çalışırken elle eklediklerin (Insert) **kaydedilmez**,
kapanınca kaybolur — kalıcı olması için dosyaya yazmak gerekir.

- **Dünya dosyası:** `/home/elifnur/Desktop/sahi_otonom/worlds/parkur.world`
- **Kural:** Her `<model name='...'>` **benzersiz** olmalı (light_model, light_model_0, light_model_1 ...).
- **Pose formatı:** `x y z roll pitch yaw` → konum metre, yönelim radyan.

---

## Yöntem 1 — Script ile (en kolay) ✅

Tek komutla ekler, otomatik yedek alır, ismi benzersiz mi kontrol eder, ilk ışığı şablon
alıp (küre lambalar + öne kaydırılmış) aynı stilde kopyalar.

```bash
cd ~/Desktop/SahiOtonom-master
python3 add_traffic_light.py <isim> <x> <y> <z> <roll> <pitch> <yaw>
```

Örnek:
```bash
python3 add_traffic_light.py light_model_4  -26.75 -0.64 2.27  3.1416 1.5516 3.1416
```

Birden fazla ışık → komutu farklı isim/konumla tekrar çalıştır:
```bash
python3 add_traffic_light.py light_model_4   10  5  2.2  0      1.5516 3.1416
python3 add_traffic_light.py light_model_5   10  8  2.2  0      1.5516 3.1416
python3 add_traffic_light.py light_model_6  -30  2  2.2  3.1416 1.5516 3.1416
```

### Pose değerlerini nereden bulurum?
1. Gazebo'yu aç (`./launch_gazebo_parkur.sh`).
2. **Insert** sekmesinden trafik ışığı modelini sahneye sürükle.
3. İstediğin yere taşı / döndür.
4. Modeli seç → sol alttaki **Property > pose** altındaki **x, y, z, roll, pitch, yaw**
   değerlerini oku.
5. Bu 6 sayıyı script'e ver. Bitti.

> İpucu: Bu modeller "yatık" tasarlandığı için dik durması adına **pitch ≈ 1.5516**
> kullanılır. **yaw** ışığın hangi yöne baktığını belirler; **roll = 0** ya da **3.1416 (π)**
> ile ışığı 180° çevirip karşı yöne baktırırsın (mevcut çiftlerde böyle yapıldı).

---

## Yöntem 2 — Elle düzenleme

1. `parkur.world` dosyasını aç.
2. Mevcut bir `<model name='light_model'> ... </model>` bloğunu komple kopyala.
3. Yapıştır (en alttaki `<state ...>` satırından **önce** bir yere).
4. İki şeyi değiştir:
   - `<model name='light_model'>` → benzersiz yeni isim, örn. `<model name='light_model_7'>`
   - Bloğun en üstündeki **model** `<pose>...</pose>` satırı → yeni `x y z roll pitch yaw`
   (İçerideki `<link>` pozlarına dokunma; onlar lambaların modele göre yerleşimi.)
5. Kaydet.

---

## Işığı kaldırmak
İlgili `<model name='light_model_X'> ... </model>` bloğunu komple sil, kaydet.

## Değişikliği görmek
```bash
./launch_gazebo_parkur.sh
```
(Script başlamadan önce eski Gazebo süreçlerini otomatik temizler, port çakışması olmaz.)

## Bir şey bozulursa
Her iki yöntem de yedek bırakır: `parkur.world.bak`. Geri almak için:
```bash
cp /home/elifnur/Desktop/sahi_otonom/worlds/parkur.world.bak \
   /home/elifnur/Desktop/sahi_otonom/worlds/parkur.world
```
