# Panduan Menjalankan Smart Trolley Navigation

Panduan ini ditujukan untuk Windows dan PowerShell. Semua perintah dijalankan
dari root repository, yaitu folder yang berisi `platformio.ini`.

## 1. Pilih yang ingin dijalankan

| Tujuan | Perintah utama | Perlu hardware |
|---|---|---|
| Melihat trolley bergerak | `python simulator\main.py` | Tidak |
| Membuka dashboard operator | `python dashboard\main.py` | Tidak untuk mode SIMULATION |
| Menjalankan semua pengujian | lihat bagian 5 | Tidak |
| Build/upload firmware ESP32-S3 | `pio run` / `pio run --target upload` | Upload perlu board |

Untuk demo pertama, jalankan **simulator desktop**. Simulator adalah cara
tercepat untuk mencoba Dijkstra, path follower, PID, replanning, dan gangguan
UWB tanpa ESP32 atau motor.

## 2. Persiapan sekali saja

Pasang:

- Python 3.9 atau lebih baru (Python 3.11 sudah diuji);
- compiler C++17: MSYS2 UCRT64 g++ atau w64devkit;
- Node.js + npm untuk aset Three.js dashboard;
- PlatformIO Core untuk tes C++ dan firmware ESP32.

Compiler diperlukan karena simulator membangun `navigation_core` C++ menjadi
`build\native\navcore.dll`. Script simulator otomatis mencari
`C:\msys64\ucrt64\bin\g++.exe`, tetapi PlatformIO native test memerlukan
`gcc.exe` dan `g++.exe` pada `PATH`. Jika compiler berada di lokasi lain,
tambahkan folder `bin`-nya untuk sesi PowerShell saat ini:

```powershell
$compilerBin = "C:\path\to\compiler\bin"
$env:PATH = "$compilerBin;$env:PATH"
$env:CXX = "$compilerBin\g++.exe"
gcc --version
g++ --version
```

Buat virtual environment dan instal semua dependency:

```powershell
# Jalankan dari root repository (folder yang berisi platformio.ini).
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r simulator\requirements.txt
python -m pip install -r dashboard\requirements.txt
python -m pip install platformio
npm install --prefix dashboard
```

Jika PowerShell melarang aktivasi script, gunakan Command Prompt dengan
`.venv\Scripts\activate.bat`, atau panggil Python langsung melalui
`.\.venv\Scripts\python.exe`.

Periksa instalasi:

```powershell
python --version
pio --version
node --version
npm --version
python scripts\validate_config.py
```

## 3. Menjalankan simulator desktop

```powershell
python simulator\main.py
```

Pada run pertama, core C++ dikompilasi otomatis. Jika kode di
`navigation_core/` berubah atau DLL bermasalah, build ulang:

```powershell
python scripts\build_native_core.py --force --verbose
```

Perintah berguna:

```powershell
python simulator\main.py --list
python simulator\main.py --scenario 5
python simulator\main.py --destination produk-susu
python simulator\main.py --headless --all
```

Kontrol utama: `Space` pause/run, `Enter` mulai/resume, klik kiri memilih tujuan,
`E` emergency stop, `C` melepas latch E-stop, `R` reset, dan `Esc` keluar.
Output run tersimpan di `logs/` (CSV) dan `results/` (JSON); keduanya adalah
hasil generate, bukan source code.

## 4. Menjalankan dashboard browser

Pastikan dependency Python dashboard dan aset npm sudah dipasang, kemudian:

```powershell
python dashboard\main.py
```

Buka <http://127.0.0.1:8000> dan gunakan mode **SIMULATION** jika tidak ada
ESP32. Tekan `Ctrl+C` di terminal untuk menghentikan server. Script ini belum
memiliki opsi CLI; `--help` juga akan menjalankan server.

Mode **HARDWARE** hanya digunakan setelah ESP32 terhubung dan port COM yang
benar dipilih. Dashboard berkomunikasi dengan firmware melalui USB serial, bukan
langsung dengan jendela simulator Pygame.

## 5. Validasi dan pengujian

Validasi cepat setelah setup:

```powershell
python scripts\validate_config.py
python -m unittest dashboard.test_models
python simulator\main.py --headless --all
pio test -e native
```

Hasil yang diharapkan pada baseline saat ini: konfigurasi valid, 4 tes model
dashboard lulus, 14/14 skenario memenuhi ekspektasi, dan 121/121 tes C++ lulus.
Skenario 11 memang sengaja berakhir `ERROR` karena tujuan tidak terjangkau; itu
dianggap lulus jika motor berhenti aman.

## 6. Build, upload, dan monitor ESP32-S3

Build firmware produksi dan versi mock-UWB:

```powershell
pio run -e esp32-s3-devkitc-1
pio run -e esp32-s3-devkitc-1-mock
```

Lihat board/port, upload, lalu buka monitor serial:

```powershell
pio device list
pio run -e esp32-s3-devkitc-1-mock --target upload --upload-port COM5
pio device monitor --port COM5 --baud 115200
```

Ganti `COM5` sesuai hasil `pio device list`. Build produksi memakai sumber posisi
`real_uwb`; untuk pengujian USB serial gunakan environment `-mock`. Di monitor:

```text
CMD:status
CMD:destination 11
CMD:start
CMD:estop
CMD:clear_estop
```

Pin ESP32, polaritas ENABLE TB6600, arah encoder, microstep DIP, arus driver, dan
dimensi roda masih harus diverifikasi pada hardware fisik. Keberhasilan build
tidak sama dengan hardware sudah aman atau sudah teruji.

## 7. Mock posisi melalui serial

Jangan membuka `pio device monitor` dan generator pada port yang sama secara
bersamaan. Jalankan generator ke firmware mock dari terminal terpisah:

```powershell
python scripts\generate_mock_position.py --port COM5 --trajectory route --noise 0.10 --dropout 0.05
```

Untuk melihat payload tanpa ESP32:

```powershell
python scripts\generate_mock_position.py --dry-run --duration 2
```

Jika ada masalah, lanjutkan ke [troubleshooting.md](troubleshooting.md).
