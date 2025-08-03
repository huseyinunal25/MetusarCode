import sys
import serial
import threading
import math
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, 
    QPushButton, QLineEdit, QLabel, QFrame
)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QPolygon, QFont
from PySide6.QtCore import QPoint

class RocketWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(500, 400)
        self.setStyleSheet("background-color: black;")
        
        self.yaw = 0
        self.pitch = 0 
        self.roll = 0
        self.altitude = 0
        
    def set_angles(self, yaw, pitch, roll):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.update()  # Yeniden çiz
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Ekran merkezi
        center_x = self.width() // 2
        center_y = self.height() // 2
        
        # Koordinat sistemini çiz
        self.draw_coordinate_system(painter, center_x, center_y)
        
        # Roketi çiz
        self.draw_rocket(painter, center_x, center_y)
        
        # Açı bilgilerini göster
        self.draw_angle_info(painter)
        
    def draw_coordinate_system(self, painter, cx, cy):
        """Koordinat sistemini çiz"""
        painter.setPen(QPen(QColor(50, 50, 50), 1))
        
        # Grid çizgileri
        for i in range(-200, 201, 40):
            painter.drawLine(cx + i, cy - 200, cx + i, cy + 200)
            painter.drawLine(cx - 200, cy + i, cx + 200, cy + i)
        
        # Ana eksenler
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.drawLine(cx - 200, cy, cx + 200, cy)  # X ekseni
        painter.drawLine(cx, cy - 200, cx, cy + 200)  # Y ekseni
        
        # Eksen etiketleri
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.setFont(QFont("Arial", 10))
        painter.drawText(cx + 180, cy - 10, "X")
        painter.drawText(cx + 10, cy - 180, "Y")
        
    def draw_rocket(self, painter, cx, cy):
        """Roketi çiz"""
        # Roket boyutu
        rocket_length = 80
        rocket_width = 20
        
        # Açıları radyana çevir
        yaw_rad = math.radians(self.yaw)
        pitch_rad = math.radians(self.pitch)
        roll_rad = math.radians(self.roll)
        
        # Roket vektörünü hesapla (pitch ve yaw'a göre)
        # Başlangıçta roket yukarı bakıyor (0, -1, 0)
        # Pitch: pozitif = burun yukarı, negatif = burun aşağı
        # Yaw: pozitif = sağa dönüş, negatif = sola dönüş
        
        # 3D roket yönü hesaplama
        rocket_dir_x = math.sin(yaw_rad) * math.cos(pitch_rad)
        rocket_dir_y = math.sin(pitch_rad)  # Pitch doğrudan Y bileşenini etkiler
        rocket_dir_z = -math.cos(yaw_rad) * math.cos(pitch_rad)
        
        # 2D görünüm için Y ve Z'yi birleştir (Z derinlik olarak kullanılıyor)
        rocket_2d_x = rocket_dir_x
        rocket_2d_y = -rocket_dir_y  # Ekranda Y ekseni ters olduğu için
        
        # Roll için yan vektörü hesapla
        side_x = math.cos(yaw_rad + math.pi/2)
        side_y = math.sin(yaw_rad + math.pi/2)
        
        # Roll açısını uygula
        rotated_side_x = side_x * math.cos(roll_rad) - 0 * math.sin(roll_rad)
        rotated_side_y = side_x * math.sin(roll_rad) + 0 * math.cos(roll_rad)
        
        # Roket gövdesi noktaları
        nose_x = cx + rocket_2d_x * rocket_length/2
        nose_y = cy + rocket_2d_y * rocket_length/2
        tail_x = cx - rocket_2d_x * rocket_length/2
        tail_y = cy - rocket_2d_y * rocket_length/2
        
        # Gövde genişliği için yan vektörler
        side_offset = rocket_width/2
        
        # Ana gövde
        painter.setPen(QPen(QColor(255, 255, 255), 3))
        painter.drawLine(int(nose_x), int(nose_y), int(tail_x), int(tail_y))
        
        # Roket burnu (üçgen)
        nose_points = QPolygon([
            QPoint(int(nose_x), int(nose_y)),
            QPoint(int(nose_x - rocket_2d_x * 15 + rotated_side_x * 8), 
                   int(nose_y - rocket_2d_y * 15 + rotated_side_y * 8)),
            QPoint(int(nose_x - rocket_2d_x * 15 - rotated_side_x * 8), 
                   int(nose_y - rocket_2d_y * 15 - rotated_side_y * 8))
        ])
        
        painter.setBrush(QBrush(QColor(255, 100, 100)))
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawPolygon(nose_points)
        
        # Gövde (dikdörtgen)
        body_points = QPolygon([
            QPoint(int(nose_x - rocket_2d_x * 15 + rotated_side_x * 8), 
                   int(nose_y - rocket_2d_y * 15 + rotated_side_y * 8)),
            QPoint(int(nose_x - rocket_2d_x * 15 - rotated_side_x * 8), 
                   int(nose_y - rocket_2d_y * 15 - rotated_side_y * 8)),
            QPoint(int(tail_x + rocket_2d_x * 10 - rotated_side_x * 8), 
                   int(tail_y + rocket_2d_y * 10 - rotated_side_y * 8)),
            QPoint(int(tail_x + rocket_2d_x * 10 + rotated_side_x * 8), 
                   int(tail_y + rocket_2d_y * 10 + rotated_side_y * 8))
        ])
        
        painter.setBrush(QBrush(QColor(200, 200, 200)))
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawPolygon(body_points)
        
        # Kanatlar
        wing_length = 25
        wing_width = 15
        
        # 4 kanat çiz
        for i in range(4):
            wing_angle = i * math.pi/2 + roll_rad
            wing_side_x = math.cos(wing_angle + yaw_rad + math.pi/2)
            wing_side_y = math.sin(wing_angle + yaw_rad + math.pi/2)
            
            wing_points = QPolygon([
                QPoint(int(tail_x + rocket_2d_x * 5), 
                       int(tail_y + rocket_2d_y * 5)),
                QPoint(int(tail_x + rocket_2d_x * 5 + wing_side_x * wing_width), 
                       int(tail_y + rocket_2d_y * 5 + wing_side_y * wing_width)),
                QPoint(int(tail_x - rocket_2d_x * wing_length + wing_side_x * wing_width), 
                       int(tail_y - rocket_2d_y * wing_length + wing_side_y * wing_width)),
                QPoint(int(tail_x - rocket_2d_x * wing_length), 
                       int(tail_y - rocket_2d_y * wing_length))
            ])
            
            painter.setBrush(QBrush(QColor(100, 150, 255)))
            painter.setPen(QPen(QColor(255, 255, 255), 1))
            painter.drawPolygon(wing_points)
        
        # Motor alevi (eğer pitch yukarı ise)
        if abs(self.pitch) < 45:  # Roket düz uçuyorsa alev göster
            flame_points = QPolygon([
                QPoint(int(tail_x), int(tail_y)),
                QPoint(int(tail_x - rocket_2d_x * 20 + rotated_side_x * 5), 
                       int(tail_y - rocket_2d_y * 20 + rotated_side_y * 5)),
                QPoint(int(tail_x - rocket_2d_x * 35), 
                       int(tail_y - rocket_2d_y * 35)),
                QPoint(int(tail_x - rocket_2d_x * 20 - rotated_side_x * 5), 
                       int(tail_y - rocket_2d_y * 20 - rotated_side_y * 5))
            ])
            
            painter.setBrush(QBrush(QColor(255, 150, 0)))
            painter.setPen(QPen(QColor(255, 200, 0), 1))
            painter.drawPolygon(flame_points)
        
        # Roket merkez noktası
        painter.setBrush(QBrush(QColor(255, 255, 0)))
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawEllipse(int(cx-3), int(cy-3), 6, 6)
        
    def draw_angle_info(self, painter):
        """Açı bilgilerini göster"""
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.setFont(QFont("Arial", 12))
        
        info_text = f"YAW: {self.yaw:6.1f}°    PITCH: {self.pitch:6.1f}°    ROLL: {self.roll:6.1f}°"
        painter.drawText(10, 25, info_text)
        
        # Yön göstergesi
        painter.setFont(QFont("Arial", 10))
        painter.drawText(10, self.height() - 60, "YAW: Sağa/Sola dönüş")
        painter.drawText(10, self.height() - 45, "PITCH: Yukarı/Aşağı")
        painter.drawText(10, self.height() - 30, "ROLL: Yan yatma")
        painter.drawText(10, self.height() - 15, "Sarı nokta: Roket merkezi")

class YerIstasyonu(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Yer İstasyonu - 3D Roket Görselleştirme")
        self.resize(1200, 700)

        self.altitude = 0

        # Ana layout
        main_layout = QHBoxLayout()

        # Sol panel (kontrol paneli)
        left_widget = QWidget()
        left_widget.setMaximumWidth(350)
        left_panel = QVBoxLayout(left_widget)
        
        # Seri port girişi
        self.port_label = QLabel("Seri Port (örnek: COM3 veya /dev/ttyUSB0):")
        self.port_input = QLineEdit()
        left_panel.addWidget(self.port_label)
        left_panel.addWidget(self.port_input)

        # Metin kutusu
        self.textbox = QTextEdit()
        self.textbox.setReadOnly(True)
        self.textbox.setMaximumHeight(200)
        left_panel.addWidget(QLabel("Gelen Veriler:"))
        left_panel.addWidget(self.textbox)

        # Bağlan butonu
        self.button = QPushButton("Bağlan")
        left_panel.addWidget(self.button)

        # Açı değerleri gösterimi
        self.angle_label = QLabel("Açı Değerleri:\nYaw: 0° (sütun 0)\nPitch: 0° (sütun 1)\nRoll: 0° (sütun 2)")
        self.angle_label.setStyleSheet("background-color: #f0f0f0; padding: 10px; border: 1px solid #ccc; color: black;")
        left_panel.addWidget(self.angle_label)

        # İrtifa göstergesi
        self.altitude_label = QLabel("İrtifa:\n0 m (sütun 1)")
        self.altitude_label.setStyleSheet("background-color: #e8f4fd; padding: 10px; border: 2px solid #2196f3; font-weight: bold; color: black;")
        left_panel.addWidget(self.altitude_label)
        

        # Test butonları
        test_label = QLabel("Test Butonları:")
        left_panel.addWidget(test_label)
        
        test_layout = QVBoxLayout()
        self.test_yaw_btn = QPushButton("Test Yaw (45°)")
        self.test_pitch_btn = QPushButton("Test Pitch (30°)")
        self.test_roll_btn = QPushButton("Test Roll (60°)")
        self.reset_btn = QPushButton("Sıfırla")
        
        test_layout.addWidget(self.test_yaw_btn)
        test_layout.addWidget(self.test_pitch_btn)
        test_layout.addWidget(self.test_roll_btn)
        test_layout.addWidget(self.reset_btn)
        
        test_widget = QWidget()
        test_widget.setLayout(test_layout)
        left_panel.addWidget(test_widget)

        # Sağ panel (3D roket görselleştirme)
        self.rocket_widget = RocketWidget()

        # Layout'ları birleştir        
        main_layout.addWidget(left_widget)
        main_layout.addWidget(self.rocket_widget)
        self.setLayout(main_layout)

        # Roket parametreleri
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        
        # Seri port değişkenleri
        self.serial_port = None
        self.running = False
        self.data_buffer = []

        # Event bağlantıları
        self.button.clicked.connect(self.connect_serial)
        self.test_yaw_btn.clicked.connect(lambda: self.test_angle('yaw', 45))
        self.test_pitch_btn.clicked.connect(lambda: self.test_angle('pitch', 30))
        self.test_roll_btn.clicked.connect(lambda: self.test_angle('roll', 60))
        self.reset_btn.clicked.connect(self.reset_angles)

        # GUI'yi güncelleyen zamanlayıcı
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(50)  # 20 FPS güncelleme

    def test_angle(self, angle_type, value):
        """Test için açı değerlerini ayarla"""
        if angle_type == 'yaw':
            self.yaw = value
        elif angle_type == 'pitch':
            self.pitch = value
        elif angle_type == 'roll':
            self.roll = value
        self.update_rocket_display()

    def reset_angles(self):
        """Tüm açıları sıfırla"""
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.update_rocket_display()

    def connect_serial(self):
        if not self.running:
            port_name = self.port_input.text().strip()
            if not port_name:
                self.textbox.append("⚠️ Lütfen seri port giriniz.")
                return
            try:
                self.serial_port = serial.Serial(port_name, 115200, timeout=1)
                self.running = True
                self.button.setText("Bağlı ✅")
                self.button.setEnabled(False)
                self.textbox.append(f"✅ {port_name} portuna bağlanıldı.")
                threading.Thread(target=self.read_serial, daemon=True).start()
            except Exception as e:
                self.textbox.append(f"❌ Bağlantı hatası: {e}")
        else:
            # Bağlantıyı kes
            self.running = False
            if self.serial_port:
                self.serial_port.close()
            self.button.setText("Bağlan")
            self.button.setEnabled(True)
            self.textbox.append("🔌 Bağlantı kesildi.")

    def read_serial(self):
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.data_buffer.append(line)
                        self.parse_angles(line)
            except Exception as e:
                self.data_buffer.append(f"Hata okuma sırasında: {e}")
                self.running = False

    def parse_angles(self, line):
        """Gelen veriden açı değerlerini çıkart"""
        try:
            # Virgülle ayrılmış değerleri ayır
            values = [val.strip() for val in line.split(',')]
            
            # Debug: veri sayısını kontrol et
            print(f"Gelen veri: {len(values)} sütun - {values}")
            
            # Kullanıcının belirlediği indeksleri al
            try:
                yaw_idx = 4
                pitch_idx = 5
                roll_idx = 6
            except ValueError:
                # Geçersiz indeks girilmişse varsayılan değerleri kullan
                yaw_idx, pitch_idx, roll_idx = 4,5,6
            
            print(f"İndeksler - Yaw:{yaw_idx}, Pitch:{pitch_idx}, Roll:{roll_idx}")
            
            # İndekslerin geçerli olup olmadığını kontrol et
            max_idx = max(yaw_idx, pitch_idx, roll_idx)
            if len(values) > max_idx and yaw_idx >= 0 and pitch_idx >= 0 and roll_idx >= 0:
                
                # Açı değerlerini ata
                old_yaw, old_pitch, old_roll = self.yaw, self.pitch, self.roll
                
                self.yaw = float(values[yaw_idx])
                self.pitch = float(values[pitch_idx])
                self.roll = float(values[roll_idx])
                self.altitude = float(values[0])
                
                print(f"Açılar güncellendi: Yaw:{old_yaw}->{self.yaw}, Pitch:{old_pitch}->{self.pitch}, Roll:{old_roll}->{self.roll}")
            else:
                print(f"Hata: Veri sayısı ({len(values)}) yetersiz, max indeks: {max_idx}")
                
        except (ValueError, IndexError) as e:
            print(f"Parse hatası: {e}")
            pass

    def update_rocket_display(self):
        """Roket görselini güncelle"""
        self.rocket_widget.set_angles(self.yaw, self.pitch, self.roll)

    def update_gui(self):
        """GUI'yi güncelle"""
        # Veri buffer'ından mesajları göster
        while self.data_buffer:
            line = self.data_buffer.pop(0)
            self.textbox.append(line)
            
            # Textbox'ı temiz tut (son 50 satır)
            if self.textbox.document().lineCount() > 50:
                cursor = self.textbox.textCursor()
                cursor.movePosition(cursor.MoveOperation.Start)
                cursor.movePosition(cursor.MoveOperation.Down, cursor.MoveMode.KeepAnchor, 10)
                cursor.removeSelectedText()
        
        
        self.angle_label.setText(f"""Açı Değerleri:
Yaw: {self.yaw:.1f}° (sütun {4})
Pitch: {self.pitch:.1f}° (sütun {5})
Roll: {self.roll:.1f}° (sütun {6})""")
        
        # İrtifa değerini güncelle
        self.altitude_label.setText(f"İrtifa:\n{self.altitude:.1f} m (sütun 1)")
        
        # Roket görselini güncelle
        self.update_rocket_display()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = YerIstasyonu()
    window.show()
    sys.exit(app.exec())