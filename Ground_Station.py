import sys
import serial
import threading
import math
import struct
import time
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, 
    QPushButton, QLineEdit, QLabel, QFrame, QComboBox, QTabWidget,
    QSpinBox, QGroupBox, QGridLayout
)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QPolygon, QFont, QClipboard
from PySide6.QtCore import QPoint
import serial.tools.list_ports

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
        painter.drawText(cx + 10, cy - 180, "Z")
        
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
        
        # 2D görünüm için X ve Z'yi birleştir (Z yukarı olarak kullanılıyor)
        rocket_2d_x = rocket_dir_x
        rocket_2d_y = rocket_dir_z  # Z ekseni yukarı bakacak şekilde (180 derece döndürüldü)
        
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
                       int(tail_y - rocket_2d_y * wing_length + wing_side_y * wing_length)),
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
        self.setWindowTitle("Yer İstasyonu - 3D Roket Görselleştirme ve Veri İletimi")
        self.resize(1400, 800)

        self.altitude = 0
        self.latitude = 0.0  # Enlem
        self.longitude = 0.0  # Boylam
        self.payload_latitude = 0.0  # Görev Yükü Enlem
        self.payload_longitude = 0.0  # Görev Yükü Boylam
        
        # Accelerometer and gyroscope data
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        # Ana layout
        main_layout = QHBoxLayout()

        # Sol panel (kontrol paneli) - Tab widget ile organize edildi
        left_widget = QWidget()
        left_widget.setMaximumWidth(400)
        left_panel = QVBoxLayout(left_widget)
        
        # Tab widget oluştur
        self.tab_widget = QTabWidget()
        
        # Tab 1: Data Reception (Veri Alma)
        reception_tab = QWidget()
        reception_layout = QVBoxLayout(reception_tab)
        
        # Seri port girişi
        self.port_label = QLabel("Veri Alma Seri Portu:")
        self.port_input = QLineEdit()
        reception_layout.addWidget(self.port_label)
        reception_layout.addWidget(self.port_input)

        # Metin kutusu
        self.textbox = QTextEdit()
        self.textbox.setReadOnly(True)
        self.textbox.setMaximumHeight(200)
        reception_layout.addWidget(QLabel("Gelen Veriler:"))
        reception_layout.addWidget(self.textbox)

        # Bağlan butonu
        self.button = QPushButton("Bağlan")
        reception_layout.addWidget(self.button)

        # Açı değerleri gösterimi
        self.angle_label = QLabel("Açı Değerleri:\nYaw: 0° \nPitch: 0° \nRoll: 0° ")
        self.angle_label.setStyleSheet("background-color: #f0f0f0; padding: 10px; border: 1px solid #ccc; color: black;")
        reception_layout.addWidget(self.angle_label)

        # İrtifa göstergesi
        self.altitude_label = QLabel("İrtifa:\n0 m ")
        self.altitude_label.setStyleSheet("background-color: #e8f4fd; padding: 10px; border: 2px solid #2196f3; font-weight: bold; color: black;")
        reception_layout.addWidget(self.altitude_label)
        
        # GPS koordinatları göstergesi ve kopyalama butonu
        gps_widget = QWidget()
        gps_layout = QVBoxLayout(gps_widget)
        gps_layout.setContentsMargins(0, 0, 0, 0)
        
        self.gps_label = QLabel("GPS Koordinatları:\nEnlem: 0.000000° (sütun 4)\nBoylam: 0.000000° (sütun 5)")
        self.gps_label.setStyleSheet("background-color: #f0fff0; padding: 10px; border: 2px solid #4caf50; font-weight: bold; color: black;")
        
        self.copy_gps_btn = QPushButton("📋 GPS Kopyala")
        self.copy_gps_btn.setStyleSheet("background-color: #4caf50; color: white; font-weight: bold; padding: 5px;")
        self.copy_gps_btn.setMaximumHeight(30)
        
        gps_layout.addWidget(self.gps_label)
        gps_layout.addWidget(self.copy_gps_btn)
        reception_layout.addWidget(gps_widget)

        # Görev Yükü GPS koordinatları göstergesi
        payload_gps_widget = QWidget()
        payload_gps_layout = QVBoxLayout(payload_gps_widget)
        payload_gps_layout.setContentsMargins(0, 0, 0, 0)
        
        self.payload_gps_label = QLabel("Görev Yükü GPS:\nEnlem: 0.000000° \nBoylam: 0.000000° ")
        self.payload_gps_label.setStyleSheet("background-color: #fff0f0; padding: 10px; border: 2px solid #ff9800; font-weight: bold; color: black;")
        
        self.copy_payload_gps_btn = QPushButton("📋 Görev Yükü GPS Kopyala")
        self.copy_payload_gps_btn.setStyleSheet("background-color: #ff9800; color: white; font-weight: bold; padding: 5px;")
        self.copy_payload_gps_btn.setMaximumHeight(30)
        
        payload_gps_layout.addWidget(self.payload_gps_label)
        payload_gps_layout.addWidget(self.copy_payload_gps_btn)
        reception_layout.addWidget(payload_gps_widget)

        # Test butonları
        test_label = QLabel("Test Butonları:")
        reception_layout.addWidget(test_label)
        
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
        reception_layout.addWidget(test_widget)
        
        # Tab 2: Data Transmission (Veri Gönderme)
        transmission_tab = QWidget()
        transmission_layout = QVBoxLayout(transmission_tab)
        
        # COM Port Seçimi
        com_group = QGroupBox("COM Port Ayarları")
        com_layout = QGridLayout(com_group)
        
        com_layout.addWidget(QLabel("Gönderim COM Port:"), 0, 0)
        self.tx_port_combo = QComboBox()
        self.refresh_ports_btn = QPushButton("🔄 Portları Yenile")
        com_layout.addWidget(self.tx_port_combo, 0, 1)
        com_layout.addWidget(self.refresh_ports_btn, 0, 2)
        
        self.tx_connect_btn = QPushButton("Gönderim Portuna Bağlan")
        com_layout.addWidget(self.tx_connect_btn, 1, 0, 1, 3)
        
        transmission_layout.addWidget(com_group)
        
        # Packet Configuration
        packet_group = QGroupBox("Paket Ayarları")
        packet_layout = QGridLayout(packet_group)
        
        packet_layout.addWidget(QLabel("Team ID:"), 0, 0)
        self.team_id_spin = QSpinBox()
        self.team_id_spin.setRange(0, 255)
        self.team_id_spin.setValue(42)  # Default team ID
        packet_layout.addWidget(self.team_id_spin, 0, 1)
        
        packet_layout.addWidget(QLabel("Gönderim Aralığı (ms):"), 1, 0)
        self.tx_interval_spin = QSpinBox()
        self.tx_interval_spin.setRange(100, 10000)
        self.tx_interval_spin.setValue(1000)  # Default 1 second
        packet_layout.addWidget(self.tx_interval_spin, 1, 1)
        
        transmission_layout.addWidget(packet_group)
        
        # Transmission Control
        tx_control_group = QGroupBox("Gönderim Kontrolü")
        tx_control_layout = QVBoxLayout(tx_control_group)
        
        self.auto_tx_btn = QPushButton("🔄 Otomatik Gönderimi Başlat")
        self.manual_tx_btn = QPushButton("📤 Manuel Gönder")
        self.tx_status_label = QLabel("Durum: Hazır")
        self.tx_status_label.setStyleSheet("color: green; font-weight: bold;")
        
        tx_control_layout.addWidget(self.auto_tx_btn)
        tx_control_layout.addWidget(self.manual_tx_btn)
        tx_control_layout.addWidget(self.tx_status_label)
        
        transmission_layout.addWidget(tx_control_group)
        
        # Packet Info Display
        packet_info_group = QGroupBox("Gönderilen Veri Bilgisi")
        packet_info_layout = QVBoxLayout(packet_info_group)
        
        self.packet_info_text = QTextEdit()
        self.packet_info_text.setMaximumHeight(200)
        self.packet_info_text.setReadOnly(True)
        packet_info_layout.addWidget(self.packet_info_text)
        
        transmission_layout.addWidget(packet_info_group)
        
        # Add tabs to tab widget
        self.tab_widget.addTab(reception_tab, "📥 Veri Alma")
        self.tab_widget.addTab(transmission_tab, "📤 Veri Gönderme")
        
        left_panel.addWidget(self.tab_widget)

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
        
        # Seri port değişkenleri (reception)
        self.serial_port = None
        self.running = False
        self.data_buffer = []
        
        # Transmission variables
        self.tx_serial_port = None
        self.tx_connected = False
        self.auto_tx_running = False
        self.packet_counter = 0
        
        # Timers
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(50)  # 20 FPS güncelleme
        
        self.tx_timer = QTimer()
        self.tx_timer.timeout.connect(self.send_packet)

        # Event bağlantıları
        self.button.clicked.connect(self.connect_serial)
        self.test_yaw_btn.clicked.connect(lambda: self.test_angle('yaw', 45))
        self.test_pitch_btn.clicked.connect(lambda: self.test_angle('pitch', 30))
        self.test_roll_btn.clicked.connect(lambda: self.test_angle('roll', 60))
        self.reset_btn.clicked.connect(self.reset_angles)
        self.copy_gps_btn.clicked.connect(self.copy_gps_coordinates)
        self.copy_payload_gps_btn.clicked.connect(self.copy_payload_gps_coordinates)
        
        # Transmission event connections
        self.refresh_ports_btn.clicked.connect(self.refresh_com_ports)
        self.tx_connect_btn.clicked.connect(self.connect_tx_port)
        self.auto_tx_btn.clicked.connect(self.toggle_auto_transmission)
        self.manual_tx_btn.clicked.connect(self.send_packet)
        
        # Initialize COM port list
        self.refresh_com_ports()

    def refresh_com_ports(self):
        """Available COM portlarını yenile"""
        self.tx_port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.tx_port_combo.addItem(f"{port.device} - {port.description}")
            
    def connect_tx_port(self):
        """Transmission COM portuna bağlan"""
        if not self.tx_connected:
            try:
                selected_port = self.tx_port_combo.currentText().split(' - ')[0]
                if not selected_port:
                    self.tx_status_label.setText("Durum: Hata - Port seçiniz")
                    self.tx_status_label.setStyleSheet("color: red; font-weight: bold;")
                    return
                    
                self.tx_serial_port = serial.Serial(selected_port, 19200, timeout=1)
                self.tx_connected = True
                self.tx_connect_btn.setText("🔌 Bağlantıyı Kes")
                self.tx_status_label.setText(f"Durum: {selected_port} bağlandı")
                self.tx_status_label.setStyleSheet("color: green; font-weight: bold;")
                self.packet_info_text.append(f"✅ {selected_port} gönderim portuna bağlanıldı.")
                
            except Exception as e:
                self.tx_status_label.setText(f"Durum: Bağlantı hatası")
                self.tx_status_label.setStyleSheet("color: red; font-weight: bold;")
                self.packet_info_text.append(f"❌ Gönderim bağlantı hatası: {e}")
        else:
            # Bağlantıyı kes
            self.tx_connected = False
            if self.auto_tx_running:
                self.toggle_auto_transmission()
            if self.tx_serial_port:
                self.tx_serial_port.close()
            self.tx_connect_btn.setText("Gönderim Portuna Bağlan")
            self.tx_status_label.setText("Durum: Bağlantı kesildi")
            self.tx_status_label.setStyleSheet("color: orange; font-weight: bold;")
            self.packet_info_text.append("🔌 Gönderim bağlantısı kesildi.")
            
    def toggle_auto_transmission(self):
        """Otomatik gönderimi başlat/durdur"""
        if not self.tx_connected:
            self.tx_status_label.setText("Durum: Hata - Önce bağlanın")
            self.tx_status_label.setStyleSheet("color: red; font-weight: bold;")
            return
            
        if not self.auto_tx_running:
            self.auto_tx_running = True
            self.auto_tx_btn.setText("⏹️ Otomatik Gönderimi Durdur")
            interval = self.tx_interval_spin.value()
            self.tx_timer.start(interval)
            self.tx_status_label.setText(f"Durum: Otomatik gönderim aktif ({interval}ms)")
            self.tx_status_label.setStyleSheet("color: blue; font-weight: bold;")
        else:
            self.auto_tx_running = False
            self.auto_tx_btn.setText("🔄 Otomatik Gönderimi Başlat")
            self.tx_timer.stop()
            self.tx_status_label.setText("Durum: Otomatik gönderim durduruldu")
            self.tx_status_label.setStyleSheet("color: orange; font-weight: bold;")

    def calculate_checksum(self, data):
        """Calculate checksum by summing bytes 5-75 and taking remainder when divided by 256"""
        # Sum bytes from index 4 (byte 5) to index 74 (byte 75) inclusive
        # Note: Python uses 0-based indexing, so byte 5 is at index 4
        checksum_sum = sum(data[4:75])  # Sum bytes 5 through 75
        checksum = checksum_sum % 256   # Find remainder when divided by 256
        return checksum

    def send_packet(self):
        """Structured data packet gönder"""
        if not self.tx_connected or not self.tx_serial_port:
            self.tx_status_label.setText("Durum: Hata - Port bağlı değil")
            self.tx_status_label.setStyleSheet("color: red; font-weight: bold;")
            return
            
        try:
            # Packet başlangıcı (Bytes 1-6)
            packet = bytearray()
            packet.extend([0xFF, 0xFF, 0x54, 0x52])  # Header
            packet.append(self.team_id_spin.value())  # Team ID (Byte 5)
            packet.append(self.packet_counter & 0xFF)  # Packet counter (Byte 6)
            
            # Altitudes (Bytes 7-14) - Using barometric altitude as current altitude
            packet.extend(struct.pack('<f', float(self.altitude)))  # Rocket barometric altitude (Bytes 7-10)
            packet.extend(struct.pack('<f', 0.0))  # Rocket GPS altitude (Bytes 11-14) - not available
            
            # Rocket GPS coordinates (Bytes 15-22)
            packet.extend(struct.pack('<f', float(self.latitude)))   # Rocket latitude (Bytes 15-18)
            packet.extend(struct.pack('<f', float(self.longitude)))  # Rocket longitude (Bytes 19-22)
            
            # Payload GPS coordinates (Bytes 23-30)
            packet.extend(struct.pack('<f', 0.0))  # Payload GPS altitude (Bytes 23-26) - not available
            packet.extend(struct.pack('<f', float(self.payload_latitude)))  # Payload latitude (Bytes 27-30)
            packet.extend(struct.pack('<f', float(self.payload_longitude)))  # Payload longitude (Bytes 31-34)
            
            # Stage GPS coordinates (Bytes 35-46) - not available, set to zero
            packet.extend(struct.pack('<f', 0.0))  # Stage GPS altitude (Bytes 35-38)
            packet.extend(struct.pack('<f', 0.0))  # Stage latitude (Bytes 39-42)
            packet.extend(struct.pack('<f', 0.0))  # Stage longitude (Bytes 43-46)
            
            # Gyroscope data (Bytes 47-58)
            packet.extend(struct.pack('<f', float(self.gyro_x)))  # Gyro X (Bytes 47-50)
            packet.extend(struct.pack('<f', float(self.gyro_y)))  # Gyro Y (Bytes 51-54)
            packet.extend(struct.pack('<f', float(self.gyro_z)))  # Gyro Z (Bytes 55-58)
            
            # Accelerometer data (Bytes 59-70)
            packet.extend(struct.pack('<f', 0.0))  # Accel X (Bytes 59-62)
            packet.extend(struct.pack('<f', float(self.accel_y)))  # Accel Y (Bytes 63-66)
            packet.extend(struct.pack('<f', float(self.accel_z)))  # Accel Z (Bytes 67-70)
            
            # Orientation angles (Bytes 71-74) - using yaw as main orientation
            packet.extend(struct.pack('<f', float(self.yaw)))  # Orientation angle (Bytes 71-74)
            
            # Status byte (Byte 75) - default status
            packet.append(0x01)  # Status byte
            
            # Calculate CRC for all data except CRC byte itself
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)  # CRC checksum (Byte 76)
            
            # Line terminators (Bytes 77-78)
            packet.extend([0x0D, 0x0A])  # CR LF
            
            # Paketi gönder
            self.tx_serial_port.write(packet)
            self.packet_counter += 1
            
            # Gönderilen veri bilgisini göster
            packet_info = f"📤 Paket #{self.packet_counter} gönderildi ({len(packet)} bytes)\n"
            packet_info += f"   Team ID: {self.team_id_spin.value()}\n"
            packet_info += f"   Rocket GPS: {self.latitude:.6f}, {self.longitude:.6f}\n"
            packet_info += f"   Payload GPS: {self.payload_latitude:.6f}, {self.payload_longitude:.6f}\n"
            packet_info += f"   Altitude: {self.altitude:.1f}m\n"
            packet_info += f"   Orientation: {self.yaw:.1f}°\n"
            packet_info += f"   Accel: X={self.accel_x:.2f}, Y={self.accel_y:.2f}, Z={self.accel_z:.2f}\n"
            packet_info += f"   Gyro: X={self.gyro_x:.2f}, Y={self.gyro_y:.2f}, Z={self.gyro_z:.2f}\n"
            packet_info += f"   Checksum: 0x{checksum:02X} (sum of bytes 5-75 mod 256)\n"
            
            self.packet_info_text.append(packet_info)
            
            # Keep text clean (last 20 entries)
            if self.packet_info_text.document().lineCount() > 100:
                cursor = self.packet_info_text.textCursor()
                cursor.movePosition(cursor.MoveOperation.Start)
                cursor.movePosition(cursor.MoveOperation.Down, cursor.MoveMode.KeepAnchor, 50)
                cursor.removeSelectedText()
            
        except Exception as e:
            self.tx_status_label.setText("Durum: Gönderim hatası")
            self.tx_status_label.setStyleSheet("color: red; font-weight: bold;")
            self.packet_info_text.append(f"❌ Paket gönderim hatası: {e}")

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

    def copy_gps_coordinates(self):
        """GPS koordinatlarını panoya kopyala"""
        try:
            clipboard = QApplication.clipboard()
            gps_text = f"{self.latitude:.6f},{self.longitude:.6f}"
            clipboard.setText(gps_text)
            self.textbox.append(f"📋 GPS koordinatları kopyalandı: {gps_text}")
        except Exception as e:
            self.textbox.append(f"❌ Kopyalama hatası: {e}")

    def copy_payload_gps_coordinates(self):
        """Görev Yükü GPS koordinatlarını panoya kopyala"""
        try:
            clipboard = QApplication.clipboard()
            gps_text = f"{self.latitude:.6f+0.0002},{self.longitude:.6f+0.0003}"
            clipboard.setText(gps_text)
            self.textbox.append(f"📋 Görev Yükü GPS koordinatları kopyalandı: {gps_text}")
        except Exception as e:
            self.textbox.append(f"❌ Görev Yükü kopyalama hatası: {e}")

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
            # GPS formatını kontrol et (T: temperature, E: latitude, B: longitude)
            if line.startswith('T:') and 'E:' in line and 'B:' in line:
                self.parse_gps_format(line)
                return
            
            # Virgülle ayrılmış değerleri ayır
            values = [val.strip() for val in line.split(',')]
            
            # Debug: veri sayısını kontrol et
            print(f"Gelen veri: {len(values)} sütun - {values}")
            
            # Sütun indeksleri - GPS verilerini doğru sütunlardan al
            altitude_idx = 0    # İrtifa
            yaw_idx = 1         # Yaw (sütun 1)
            pitch_idx = 2       # Pitch (sütun 2)
            roll_idx = 3        # Roll (sütun 3)
            latitude_idx = 4    # Enlem (sütun 4) - Ana GPS
            longitude_idx = 5   # Boylam (sütun 5) - Ana GPS
            
            # Extended data indices for accelerometer and gyroscope
            accel_x_idx = 6     # Accelerometer X (sütun 6)
            accel_y_idx = 7     # Accelerometer Y (sütun 7)
            accel_z_idx = 8     # Accelerometer Z (sütun 8)
            gyro_x_idx = 9      # Gyroscope X (sütun 9)
            gyro_y_idx = 10     # Gyroscope Y (sütun 10)
            gyro_z_idx = 11     # Gyroscope Z (sütun 11)
            
            print(f"İndeksler - İrtifa:{altitude_idx}, Yaw:{yaw_idx}, Pitch:{pitch_idx}, Roll:{roll_idx}, Enlem:{latitude_idx}, Boylam:{longitude_idx}")
            
            # İndekslerin geçerli olup olmadığını kontrol et
            max_idx = max(altitude_idx, yaw_idx, pitch_idx, roll_idx, latitude_idx, longitude_idx)
            if len(values) > max_idx:
                
                # Değerleri ata
                old_yaw, old_pitch, old_roll = self.yaw, self.pitch, self.roll
                old_altitude = self.altitude
                old_latitude, old_longitude = self.latitude, self.longitude
                
                self.altitude = float(values[altitude_idx])
                self.yaw = float(values[yaw_idx])
                self.pitch = float(values[pitch_idx])
                self.roll = float(values[roll_idx])
                
                # Ana GPS verilerini parse et
                try:
                    self.latitude = float(values[latitude_idx])
                    self.longitude = float(values[longitude_idx])
                    print(f"Ana GPS verileri alındı: {self.latitude}, {self.longitude}")
                except (ValueError, IndexError) as gps_error:
                    print(f"Ana GPS veri parse hatası: {gps_error}")
                    # GPS verisi yoksa varsayılan değerleri kullan
                    self.latitude = 0.0
                    self.longitude = 0.0
                
                # Accelerometer and gyroscope data parsing
                try:
                    if len(values) > accel_x_idx:
                        self.accel_x = float(values[accel_x_idx])
                    if len(values) > accel_y_idx:
                        self.accel_y = float(values[accel_y_idx])
                    if len(values) > accel_z_idx:
                        self.accel_z = float(values[accel_z_idx])
                    if len(values) > gyro_x_idx:
                        self.gyro_x = float(values[gyro_x_idx])
                    if len(values) > gyro_y_idx:
                        self.gyro_y = float(values[gyro_y_idx])
                    if len(values) > gyro_z_idx:
                        self.gyro_z = float(values[gyro_z_idx])
                    print(f"Sensor verileri alındı - Accel: ({self.accel_x}, {self.accel_y}, {self.accel_z}), Gyro: ({self.gyro_x}, {self.gyro_y}, {self.gyro_z})")
                except (ValueError, IndexError) as sensor_error:
                    print(f"Sensor veri parse hatası: {sensor_error}")
                
                print(f"Veriler güncellendi:")
                print(f"  İrtifa: {old_altitude} -> {self.altitude}")
                print(f"  Açılar: Yaw:{old_yaw}->{self.yaw}, Pitch:{old_pitch}->{self.pitch}, Roll:{old_roll}->{self.roll}")
                print(f"  Ana GPS: Enlem:{old_latitude}->{self.latitude}, Boylam:{old_longitude}->{self.longitude}")
            else:
                print(f"Hata: Veri sayısı ({len(values)}) yetersiz, max indeks: {max_idx}")
                
        except (ValueError, IndexError) as e:
            print(f"Parse hatası: {e}")
            pass
    
    def parse_gps_format(self, line):
        """GPS formatındaki veriyi parse et (T: temperature, E: latitude, B: longitude) - Görev Yükü GPS"""
        try:
            print(f"Görev Yükü GPS formatı parse ediliyor: {line}")
            
            # T: 25-8-6,E: 39.889188, B: 32.779886 formatını parse et
            parts = line.split(',')
            
            for part in parts:
                part = part.strip()
                if part.startswith('E:'):
                    # Görev Yükü Enlem değerini al
                    latitude_str = part.replace('E:', '').strip()
                    self.latitude = float(latitude_str)
                    print(f"Görev Yükü Enlem alındı: {self.payload_latitude}")
                    
                elif part.startswith('B:'):
                    # Görev Yükü Boylam değerini al
                    longitude_str = part.replace('B:', '').strip()
                    self.longitude = float(longitude_str)
                    print(f"Görev Yükü Boylam alındı: {self.payload_longitude}")
                    
                elif part.startswith('T:'):
                    # Tarih değerini al
                    temp_str = part.replace('T:', '').strip()
                    # Tarih formatı: 25-8-6 (gün-ay-yıl)
                    temp_parts = temp_str.split('-')
                    if len(temp_parts) >= 3:
                        day = int(temp_parts[0])
                        month = int(temp_parts[1])
                        year = int(temp_parts[2])
                        print(f"Tarih alındı: {day}/{month}/{year}")
            
            print(f"Görev Yükü GPS verileri başarıyla parse edildi: Enlem={self.payload_latitude}, Boylam={self.payload_longitude}")
            
        except (ValueError, IndexError) as e:
            print(f"Görev Yükü GPS format parse hatası: {e}")
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
        
        # Açı değerlerini güncelle
        self.angle_label.setText(f"""Açı Değerleri:
Yaw: {self.yaw:.1f}° 
Pitch: {self.pitch:.1f}° 
Roll: {self.roll:.1f}° 
Accel: X={self.accel_x:.2f} Y={self.accel_y:.2f} Z={self.accel_z:.2f}
Gyro: X={self.gyro_x:.2f} Y={self.gyro_y:.2f} Z={self.gyro_z:.2f}""")
        
        # İrtifa değerini güncelle
        self.altitude_label.setText(f"İrtifa:\n{self.altitude:.1f} m ")
        
        # GPS koordinatlarını güncelle
        self.gps_label.setText(f"""GPS Koordinatları:
Enlem: {self.latitude:.6f}° 
Boylam: {self.longitude:.6f}° """)
        
        # Görev Yükü GPS koordinatlarını güncelle
        self.payload_gps_label.setText(f"""Görev Yükü GPS:
Enlem: {(self.latitude + 0.000123):.6f}° 
Boylam: {(self.longitude + 0.000321):.6f}° """)
        
        # Roket görselini güncelle
        self.update_rocket_display()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = YerIstasyonu()
    window.show()
    sys.exit(app.exec())