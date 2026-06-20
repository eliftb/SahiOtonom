# Git Katkı Rehberi

Bu belge SahiOtonom projesinde kod değişikliklerinin nasıl yönetileceğini açıklar.

---

## Branch Modeli — GitHub Flow

```
master  ──────────────────────────────────────────────────►  (her zaman çalışır)
           │              │              │
           ▼              ▼              ▼
    feature/xxx      fix/yyy       hotfix/zzz
        │                │              │
        └── PR aç ───────┴──────────────┘
                 incelenir → onaylanır → master'a merge
```

`master` branch'i **her zaman stabil ve çalışır** olmalıdır. Yarışma öncesinde doğrudan `master`'dan çalışılır. Doğrudan `master`'a push **yasaktır**.

---

## Branch Adlandırma

| Tür | Format | Örnek |
|-----|--------|-------|
| Yeni özellik | `feature/<kısa-açıklama>` | `feature/kesisim-tespiti` |
| Hata düzeltme | `fix/<kısa-açıklama>` | `fix/pid-integral-hatasi` |
| Acil/kritik düzeltme | `hotfix/<kısa-açıklama>` | `hotfix/uart-port-crash` |
| Refactor / temizlik | `refactor/<kısa-açıklama>` | `refactor/config-dataclass` |
| Test ekleme | `test/<kısa-açıklama>` | `test/lateral-deviation` |
| Dokümantasyon | `docs/<kısa-açıklama>` | `docs/git-rehberi` |

**Kurallar:**
- Küçük harf, kelimeler tire ile ayrılır
- Türkçe karakter kullanma (`ş` → `s`, `ğ` → `g`, vb.)
- Kısa ve açıklayıcı olsun (2-4 kelime)

---

## Commit Mesajı Formatı

> **Tüm commit mesajları İngilizce yazılmalıdır.**

```
<tip>: <ne yapıldı — kısa ve öz, İngilizce>

[isteğe bağlı: neden yapıldı veya ek bağlam]
```

### Tip Seçenekleri

| Tip | Ne zaman |
|-----|----------|
| `feat` | Yeni özellik eklendi |
| `fix` | Hata düzeltildi |
| `refactor` | Davranış değişmeden kod yeniden yapılandırıldı |
| `docs` | Yalnızca dokümantasyon değişti |
| `test` | Test eklendi veya düzenlendi |
| `chore` | Bağımlılık güncelleme, config değişikliği vb. |
| `perf` | Performans iyileştirmesi |

### Örnekler

```bash
# Doğru — İngilizce
git commit -m "feat: add intersection_direction topic for junction detection"
git commit -m "fix: PID integral was missing dt normalization"
git commit -m "refactor: move serial port config to .env"
git commit -m "docs: add CV_DISPLAY section to utils README"

# Yanlış
git commit -m "fix"              # çok kısa
git commit -m "update"           # belirsiz
git commit -m "wip"              # yarım iş commit'lenmez
git commit -m "düzeltme"         # Türkçe — yasak
```

**Kurallar:**
- Emir kipi kullan: "add feature" doğru, "added feature" yanlış
- 72 karakteri geçme
- Spesifik ol: "fix: correct angle calculation in obstacle detection", "fix bug" değil

---

## PR (Pull Request) Akışı

### 1. Branch aç

```bash
git checkout master
git pull origin master                        # master'ı güncelle
git checkout -b feature/yeni-ozellik          # yeni branch
```

### 2. Değişikliklerini yap ve commit et

```bash
git add <dosyalar>                            # spesifik dosyaları ekle, git add . kullanma
git commit -m "feat: short english description"
git push origin feature/yeni-ozellik
```

### 3. PR aç

GitHub üzerinden PR açarken aşağıdaki şablonu kullan:

```
## Ne değişti?
- [ ] Kısaca yaptığın değişikliği açıkla

## Neden değişti?
- Motivasyonu ve bağlamı açıkla

## Nasıl test edildi?
- [ ] Node manuel olarak çalıştırıldı
- [ ] Simüle edilmiş veriyle test edildi
- [ ] Gerçek araç üzerinde test edildi

## Dikkat edilmesi gerekenler
- Varsa yan etkiler, bilinen sorunlar veya inceleyiciye notlar
```

### 4. İnceleme süreci

- Atanan inceleyici kodu gözden geçirir
- Gerekirse yorum yazar, değişiklik ister
- **En az 1 onay** gereklidir
- Onay sonrası PR sahibi veya inceleyici merge eder

### 5. Merge stratejisi

Tüm PR'lar **Squash and Merge** ile birleştirilir:
- `master` geçmişi temiz ve okunabilir kalır
- Geliştirme sırasındaki "wip", ara commit'ler gizlenir
- Merge sonrası branch silinir

---

## İnceleyici Kontrol Listesi

PR'ı incelerken şunları kontrol et:

- [ ] Kod çalışıyor mu? (varsa test çıktısı paylaşıldı mı?)
- [ ] Yeni yüksek frekanslı log'lar `debug()` seviyesinde mi?
- [ ] Hardcode değer kalmış mı? (port, eşik, PID değeri — `.env`'e taşınmalı)
- [ ] `imshow` / `waitKey` çağrıları `self._cv_display` ile korunuyor mu?
- [ ] `.env` dosyasında yeni anahtar varsa `.env.example`'a da eklendi mi?
- [ ] `requirements.txt` güncellenmesi gerekiyor mu?
- [ ] Değişiklik başka bir node'u etkiliyor mu?

---

## Hızlı Referans

```bash
# Güncel master'dan yeni branch aç
git checkout master && git pull && git checkout -b feature/isim

# Değişiklikleri gözden geçir (hangi dosyalar değişti)
git status
git diff

# Sadece ilgili dosyaları ekle (git add . kullanma)
git add SeritTespit/serit-tespitcopy.py utils/config.py

# Commit (İngilizce!)
git commit -m "feat: short english description"

# Push
git push origin feature/isim

# Branch'ini master ile güncelle (merge conflict öncesi)
git fetch origin
git rebase origin/master

# Merge edilmiş branch'i temizle
git branch -d feature/isim
```

---

## Sık Yapılan Hatalar

| Hata | Çözüm |
|------|-------|
| Doğrudan `master`'a push | Branch aç, PR kullan |
| Türkçe commit mesajı | İngilizce yaz — `git commit --amend` ile düzelt |
| `git add .` ile `.env` stage'lendi | `.gitignore` kontrol et; `git rm --cached .env` |
| `*.pt` model dosyası commit'lendi | `.gitignore` kontrol et; büyük dosyaları eklemekten kaçın |
| Aynı branch üzerinde uzun süre çalışma | Sık sık `rebase origin/master` ile güncelle |
| Tek commit'te birden fazla konu | Konuları ayır, ayrı commit'ler veya ayrı PR'lar aç |
