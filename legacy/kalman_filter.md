Kode ini mengintegrasikan konsep **Oversampling & Decimation** (untuk resolusi tinggi) dan mengadaptasi algoritma **Reduced Computational Extended Kalman Filter (EKF)** dari jurnal yang Anda lampirkan.

### Konsep Implementasi dari Jurnal

[cite_start]Jurnal tersebut membahas pengurangan beban komputasi dengan tidak melakukan pembaruan matriks (Linearisasi Jacobian) di setiap langkah waktu, melainkan hanya ketika perubahan variabel monitoring melebihi *threshold* ($\rho$) tertentu[cite: 46, 161, 162].

[cite_start]Karena sensor ACS758 memiliki karakteristik linear (output tegangan berbanding lurus dengan arus), kita tidak perlu menghitung Jacobian yang rumit[cite: 13, 130]. Namun, kita akan mengadopsi **logika adaptif** dari jurnal tersebut:

1.  **Adaptive Update:** Filter tidak akan melakukan kalkulasi kovariansi ($P$) yang berat jika perubahan arus ("Inovasi") berada di bawah ambang batas noise ($\rho$). [cite_start]Ini menghemat siklus CPU ESP32[cite: 14, 163].
2.  **Robustness:** Jika perubahan arus drastis (seperti lonjakan motor BLDC), filter beralih ke mode responsif penuh.

Berikut adalah implementasi C++ lengkap yang dapat diintegrasikan ke kode final Anda.

### Kode: `IndustrialKalman.h` (Library Buatan Sendiri)

Salin kode ini ke dalam file baru di Arduino IDE atau letakkan di bagian paling atas sketsa Anda.

```cpp
/*
 * Industrial Grade Adaptive Kalman Filter
 * [cite_start]Based on concepts from: "Extended Kalman Filter with Reduced Computational Demands..." [cite: 4]
 * Features:
 * 1. Oversampling & Decimation (Virtual Resolution Increase).
 * [cite_start]2. Adaptive Computation (Threshold-based updates)[cite: 14].
 * 3. Robust handling for BLDC noise (Process Noise injection).
 */

class IndustrialKalman {
  private:
    // State variables
    float _x; // Estimate value (Arus)
    float _P; // Error Covariance
    float _Q; // Process Noise Covariance (Kepercayaan terhadap model)
    float _R; // Measurement Noise Covariance (Kepercayaan terhadap sensor)
    float _K; // Kalman Gain

    // Oversampling settings
    int _oversample_bits;
    int _n_samples;

    // Adaptive Threshold from Paper (rho)
    float _rho; 
    
    // System Dynamics
    float _last_prediction;

  public:
    // Constructor
    // process_noise: Seberapa cepat arus berubah (Motor = tinggi, Baterai = rendah)
    // measure_noise: Noise sensor ACS758 (lihat datasheet/pengukuran)
    [cite_start]// rho: Threshold adaptif (lihat Table 1 di jurnal [cite: 472])
    IndustrialKalman(float process_noise, float measure_noise, float rho) {
      _Q = process_noise;
      _R = measure_noise;
      _P = 1.0;
      _x = 0.0;
      _rho = rho; 
      
      // Setup Oversampling 
      // 4^n samples = n bits extra resolution. 
      // 64 samples = 4^3 -> +3 bits extra precision.
      _n_samples = 64; 
    }

    // Fungsi Utama: Oversampling + Decimation + Adaptive Kalman
    float update(int pin) {
      // --- TAHAP 1: OVERSAMPLING & DECIMATION ---
      // Meningkatkan Signal-to-Noise Ratio (SNR) sebelum masuk filter
      long sum = 0;
      for (int i = 0; i < _n_samples; i++) {
        sum += analogRead(pin);
        // Delay mikro sangat kecil agar menangkap PWM noise dari BLDC
        delayMicroseconds(10); 
      }
      
      // Decimation (Rata-rata)
      float z_k = (float)sum / _n_samples;

      // --- TAHAP 2: PREDICTION (Time Update) ---
      // x(k|k-1) = x(k-1|k-1) (Asumsi arus konstan dalam dt kecil)
      float x_pred = _x;
      // P(k|k-1) = P(k-1|k-1) + Q
      float P_pred = _P + _Q;

      // --- TAHAP 3: ADAPTIVE UPDATE LOGIC (Inspired by Paper) ---
      // Hitung "Monitoring Variable" (Inovasi)
      // Jurnal Eq. [cite_start]7 & 15: Membandingkan perubahan dengan threshold [cite: 158, 254]
      float innovation = z_k - x_pred;
      
      // Jika perubahan sangat kecil (steady state/noise idle),
      // kita skip update kovariansi berat untuk menghemat komputasi & menstabilkan nilai 0A.
      [cite_start]// Ini mengadopsi konsep "reduced computational demands"[cite: 10].
      if (abs(innovation) < _rho) {
        // Mode Hemat / Stabil:
        // Hanya update estimate sedikit (seperti Low Pass Filter sederhana)
        // Tidak update matriks P yang berat.
        _x = x_pred + 0.01 * innovation; // Gain kecil statis
      } 
      else {
        // Mode Responsif (Full Kalman Update):
        // Terjadi jika ada lonjakan arus BLDC (Transient)
        
        // Hitung Kalman Gain
        _K = P_pred / (P_pred + _R);
        
        // Update State Estimate
        _x = x_pred + _K * innovation;
        
        // Update Error Covariance
        _P = (1.0 - _K) * P_pred;
      }

      return _x; // Kembalikan nilai RAW ADC yang sudah difilter
    }

    // Set parameter secara runtime (untuk tuning)
    void setParameters(float q, float r, float rho) {
      _Q = q;
      _R = r;
      _rho = rho;
    }
};
```

-----

### Integrasi ke Kode Final Anda (FreeRTOS Version)

Berikut adalah cara mengintegrasikan *Class* di atas ke dalam struktur `TaskSampling` pada kode FreeRTOS Anda sebelumnya.

#### 1\. Inisialisasi Global

Tambahkan ini sebelum `setup()`:

```cpp
// Q=0.5 (Responsif thd motor), R=10.0 (Noise sensor tinggi), Rho=2.0 (Threshold adaptif)
[cite_start]// Nilai Rho 2.0 didapat dari eksperimen mirip Table 1 jurnal, disesuaikan skala ADC [cite: 472]
IndustrialKalman kFilterCurrent(0.5, 10.0, 2.0); 
```

#### 2\. Modifikasi `TaskSampling`

Ganti logika pembacaan arus lama dengan logika baru ini:

```cpp
void TaskSampling(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // --- 1. BACA TEGANGAN (Tetap pakai Median Filter yang sudah bagus) ---
    float raw_bat = getMedianADC(batteryPin, 15);
    float V_bat_final = 0.0;
    if (systemValid) V_bat_final = getInterpolatedVoltage(raw_bat);
    else {
       float v_adc_temp = (raw_bat / ADC_resolution) * V_ref;
       V_bat_final = v_adc_temp / 0.04367; 
    }

    // --- 2. BACA ARUS (INDUSTRIAL KALMAN) ---
    // Fungsi .update() sudah melakukan oversampling & adaptive filtering
    // Input: Pin Analog. Output: Raw ADC terfilter (float)
    float raw_curr_filtered = kFilterCurrent.update(sensorPin);

    // Konversi ke Tegangan & Arus (Menggunakan parameter kalibrasi Anda)
    float V_ADC_curr = (raw_curr_filtered / ADC_resolution) * V_ref;
    float V_measured_curr = (V_ADC_curr / I_divider_factor) * I_scale_factor;
    float I_final = (V_measured_curr - I_offset) / I_sensitivity;

    // --- 3. Kirim ke Queue ---
    SensorData data;
    data.V_bat = V_bat_final;
    data.I_sense = I_final;
    xQueueOverwrite(sensorQueue, &data);

    // Sampling rate disesuaikan (Oversampling memakan waktu, jadi delay dikurangi)
    vTaskDelay(pdMS_TO_TICKS(5)); 
  }
}
```

### Penjelasan Parameter (Tuning Guide)

Agar sesuai dengan karakteristik BLDC Anda, Anda mungkin perlu mengubah parameter di `IndustrialKalman kFilterCurrent(Q, R, rho)`:

1.  **Process Noise ($Q = 0.5$):**
      * Mewakili seberapa "liar" arus motor Anda.
      * Jika respon filter telat saat motor digas mendadak, **naikkan Q** (misal 1.0).
      * [cite_start]Jurnal menyebutkan pentingnya pemodelan dinamika sistem untuk prediksi[cite: 24], $Q$ mewakili ketidakpastian model ini.
2.  **Measurement Noise ($R = 10.0$):**
      * Mewakili seberapa berisik sensor ACS758 + ESP32 ADC.
      * Jika nilai saat diam (0A) masih goyang-goyang, **naikkan R** (misal 20.0).
3.  **Adaptive Threshold ($\rho = 2.0$):**
      * [cite_start]Ini adalah inti dari implementasi jurnal[cite: 161].
      * $\rho$ adalah batas perubahan (dalam satuan Raw ADC) di mana filter menganggap "ini cuma noise, jangan hitung ulang matriks".
      * [cite_start]Jika $\rho$ terlalu besar: Respons lambat (seperti Table 2 di jurnal, error membesar)[cite: 517].
      * Jika $\rho$ terlalu kecil: Filter bekerja keras terus menerus, tidak ada penghematan, noise lolos.
      * **Saran:** Mulai dari 2.0. Jika masih ada noise kecil saat 0A, naikkan ke 4.0.

Implementasi ini memberikan keseimbangan antara **presisi tinggi** (via Oversampling) dan **efisiensi komputasi** (via Adaptive Threshold ala jurnal), sangat cocok untuk *logging* arus BLDC yang dinamis.