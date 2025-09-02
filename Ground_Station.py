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
        self.gpsaltitude = 0.0

        
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
        roll_rad = math.radians(self.yaw)
        pitch_rad = math.radians(self.pitch)
        yaw_rad = math.radians(self.roll)
        
        # Roket vektörünü hesapla (pitch ve yaw'a göre)
        # Başlangıçta roket yukarı bakıyor (0, -1, 0)
        # Pitch: pozitif = burun yukarı, negatif = burun aşağı
        # Yaw: pozitif = sağa dönüş, negatif = sola dönüş
        # Roll: pozitif = sağa yatma, negatif = sola yatma
        
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
        

class YerIstasyonu(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Yer İstasyonu - 3D Roket Görselleştirme ve Veri İletimi")
        self.resize(1400, 700)

        self.altitude = 0
        self.latitude = 0.0  # Enlem
        self.longitude = 0.0  # Boylam
        self.gpsaltitude = 0.0

        self.payload_altitude = 0.0
        self.payload_latitude = 0.0  # Görev Yükü Enlem
        self.payload_longitude = 0.0  # Görev Yükü Boylam
        self.payload_strain = 0.0
        self.payload_bmealtitude = 0.0
        self.payload_pressure = 0.0
        self.payload_humidity = 0.0
        self.payload_temperature = 0.0

        
        # Accelerometer and gyroscope data
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        self.angle_z = 0.0
        
        # Status byte
        self.status_byte = 0x00

        # Ana layout
        main_layout = QHBoxLayout()

        # Sol panel (kontrol paneli) - Tab widget ile organize edildi
        left_widget = QWidget()
        left_widget.setMaximumWidth(600)
        left_panel = QVBoxLayout(left_widget)
        
        # Tab widget oluştur
        self.tab_widget = QTabWidget()
        
        # Tab 1: Data Reception (Veri Alma)
        reception_tab = QWidget()
        reception_layout = QVBoxLayout(reception_tab)
        
        # Seri port girişi
        rocket_group = QGroupBox("Roket Veri Alma")
        rocket_layout = QGridLayout(rocket_group)
        
        rocket_layout.addWidget(QLabel("Roket COM Port:"), 0, 0)
        self.rocket_port_combo = QComboBox()
        self.rocket_refresh_btn = QPushButton("🔄 Portları Yenile")
        rocket_layout.addWidget(self.rocket_port_combo, 0, 1)
        rocket_layout.addWidget(self.rocket_refresh_btn, 0, 2)
        
        self.rocket_connect_btn = QPushButton("Roket Portuna Bağlan")
        rocket_layout.addWidget(self.rocket_connect_btn, 1, 0, 1, 3)
        
        reception_layout.addWidget(rocket_group)
        
        # Payload COM port selection
        payload_group = QGroupBox("Görev Yükü (Payload) GPS")
        payload_layout = QGridLayout(payload_group)
        
        payload_layout.addWidget(QLabel("Payload COM Port:"), 0, 0)
        self.payload_port_combo = QComboBox()
        self.payload_refresh_btn = QPushButton("🔄 Portları Yenile")
        payload_layout.addWidget(self.payload_port_combo, 0, 1)
        payload_layout.addWidget(self.payload_refresh_btn, 0, 2)
        
        self.payload_connect_btn = QPushButton("Payload Portuna Bağlan")
        payload_layout.addWidget(self.payload_connect_btn, 1, 0, 1, 3)
        
        reception_layout.addWidget(payload_group)



        # Açı değerleri gösterimi
        self.angle_label = QLabel("Açı Değerleri:\nYaw: 0° \nPitch: 0° \nRoll: 0° ")
        self.angle_label.setStyleSheet("background-color: #f0f0f0; padding: 10px; border: 1px solid #ccc; color: black;")

        # İrtifa göstergesi
        self.altitude_label = QLabel("İrtifa:\n0 m\nGPS İrtifa:\n0 m")
        self.altitude_label.setStyleSheet(
            "background-color: #e8f4fd; padding: 10px; border: 2px solid #2196f3; font-weight: bold; color: black;"
        )

        # Açı + İrtifa yan yana
        angle_alt_layout = QHBoxLayout()
        angle_alt_layout.addWidget(self.angle_label)
        angle_alt_layout.addWidget(self.altitude_label)
        reception_layout.addLayout(angle_alt_layout)

        
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

        # Görev Yükü GPS koordinatları göstergesi
        payload_gps_widget = QWidget()
        payload_gps_layout = QVBoxLayout(payload_gps_widget)
        payload_gps_layout.setContentsMargins(0, 0, 0, 0)
        
        self.payload_gps_label = QLabel("Görev Yükü GPS:\nEnlem: 0.000000° \nBoylam: 0.000000° \nStrain: 0.000000")
        self.payload_gps_label.setStyleSheet("background-color: #fff0f0; padding: 10px; border: 2px solid #ff9800; font-weight: bold; color: black;")
        
        self.copy_payload_gps_btn = QPushButton("📋 Görev Yükü GPS Kopyala")
        self.copy_payload_gps_btn.setStyleSheet("background-color: #ff9800; color: white; font-weight: bold; padding: 5px;")
        self.copy_payload_gps_btn.setMaximumHeight(30)
        
        payload_gps_layout.addWidget(self.payload_gps_label)
        payload_gps_layout.addWidget(self.copy_payload_gps_btn)

        gps_pair_layout = QHBoxLayout()
        gps_pair_layout.addWidget(gps_widget)
        gps_pair_layout.addWidget(payload_gps_widget)
        reception_layout.addLayout(gps_pair_layout)

        
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
    
        
        transmission_layout.addWidget(packet_group)
        
        # Transmission Control
        tx_control_group = QGroupBox("Gönderim Kontrolü")
        tx_control_layout = QVBoxLayout(tx_control_group)
        
        self.auto_tx_btn = QPushButton("🔄 Otomatik Gönderimi Başlat")
        self.tx_status_label = QLabel("Durum: Hazır")
        self.tx_status_label.setStyleSheet("color: green; font-weight: bold;")
        
        tx_control_layout.addWidget(self.auto_tx_btn)
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
        
        # Ring buffer for packet synchronization
        self.ring_buffer = bytearray(1024)  # 1KB ring buffer
        self.ring_buffer_head = 0
        self.ring_buffer_tail = 0
        self.ring_buffer_size = 0
        
        # Payload ring buffer
        self.payload_ring_buffer = bytearray(1024)
        self.payload_ring_head = 0
        self.payload_ring_tail = 0
        self.payload_ring_size = 0


        # Payload serial port variables
        self.payload_serial_port = None
        self.payload_running = False
        self.payload_data_buffer = []
        
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
        self.rocket_connect_btn.clicked.connect(self.connect_rocket_serial)
        self.copy_gps_btn.clicked.connect(self.copy_gps_coordinates)
        self.copy_payload_gps_btn.clicked.connect(self.copy_payload_gps_coordinates)
        
        # Rocket port event connections
        self.rocket_refresh_btn.clicked.connect(self.refresh_rocket_ports)
        
        # Payload port event connections
        self.payload_refresh_btn.clicked.connect(self.refresh_payload_ports)
        self.payload_connect_btn.clicked.connect(self.connect_payload_port)
        
        # Transmission event connections
        self.refresh_ports_btn.clicked.connect(self.refresh_com_ports)
        self.tx_connect_btn.clicked.connect(self.connect_tx_port)
        self.auto_tx_btn.clicked.connect(self.toggle_auto_transmission)
        
        # Initialize COM port list
        self.refresh_com_ports()
        self.refresh_rocket_ports()
        self.refresh_payload_ports()

    def calculate_z_angle_from_accel(self, ax, ay, az):
        mag = math.sqrt(ax*ax + ay*ay + az*az)
        if mag < 1e-6:
            return 0.0
        dot = az / mag
        dot = max(-1.0, min(1.0, dot))  # clamp
        return math.degrees(math.acos(dot))


    def refresh_com_ports(self):
        """Available COM portlarını yenile"""
        self.tx_port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.tx_port_combo.addItem(f"{port.device} - {port.description}")
            
    def refresh_rocket_ports(self):
        """Available COM portlarını rocket için yenile"""
        self.rocket_port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.rocket_port_combo.addItem(f"{port.device} - {port.description}")
            
    def refresh_payload_ports(self):
        """Available COM portlarını payload için yenile"""
        self.payload_port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.payload_port_combo.addItem(f"{port.device} - {port.description}")
            
    def connect_payload_port(self):
        """Payload COM portuna bağlan"""
        if not self.payload_running:
            try:
                selected_port = self.payload_port_combo.currentText().split(' - ')[0]
                if not selected_port:
                    return
                    
                self.payload_serial_port = serial.Serial(selected_port, 115200, timeout=1)
                self.payload_running = True
                self.payload_connect_btn.setText("🔌 Payload Bağlantıyı Kes")
                threading.Thread(target=self.read_payload_serial, daemon=True).start()
                
            except Exception as e:
                pass
        else:
            # Bağlantıyı kes
            self.payload_running = False
            if self.payload_serial_port:
                self.payload_serial_port.close()
            self.payload_connect_btn.setText("Payload Portuna Bağlan")
            
    def read_payload_serial(self):
        """Payload seri portundan veri oku (ring buffer ile)"""
        while self.payload_running:
            try:
                if self.payload_serial_port and self.payload_serial_port.in_waiting:
                    data = self.payload_serial_port.read(self.payload_serial_port.in_waiting)
                    if data:
                        self.add_to_payload_ring_buffer(data)
                        self.process_payload_ring_buffer_packets()
            except Exception as e:
                self.payload_data_buffer.append(f"Payload okuma hatası: {e}")
                self.payload_running = False

            
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
            interval = 200 
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
            packet.extend(struct.pack('<f', float(self.gpsaltitude)))  # Rocket GPS altitude (Bytes 11-14) 
            
            # Rocket GPS coordinates (Bytes 15-22)
            packet.extend(struct.pack('<f', float(self.latitude)))   # Rocket latitude (Bytes 15-18)
            packet.extend(struct.pack('<f', float(self.longitude)))  # Rocket longitude (Bytes 19-22)
            
            # Payload GPS coordinates (Bytes 23-30)
            packet.extend(struct.pack('<f', float(self.payload_altitude)))  # Payload GPS altitude (Bytes 23-26) - not available
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
            packet.extend(struct.pack('<f', float(self.accel_x)))  # Accel X (Bytes 59-62)
            packet.extend(struct.pack('<f', float(self.accel_y)))  # Accel Y (Bytes 63-66)
            packet.extend(struct.pack('<f', float(self.accel_z)))  # Accel Z (Bytes 67-70)
            
            # Orientation angles (Bytes 71-74) - using yaw as main orientation
            packet.extend(struct.pack('<f', float(self.angle_z)))  # Orientation angle (Bytes 71-74)
            
            # Status byte (Byte 75) - default status
            packet.append(self.map_status_byte(self.status_byte))
            
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

        if self.packet_counter == 255:
            self.packet_counter = 0

    def copy_gps_coordinates(self):
        """GPS koordinatlarını panoya kopyala"""
        try:
            clipboard = QApplication.clipboard()
            gps_text = f"{self.latitude:.6f},{self.longitude:.6f}"
            clipboard.setText(gps_text)
        except Exception as e:
            pass

    def copy_payload_gps_coordinates(self):
        """Görev Yükü GPS koordinatlarını panoya kopyala"""
        try:
            clipboard = QApplication.clipboard()
            gps_text = f"{self.payload_latitude:.6f},{self.payload_longitude:.6f}"
            clipboard.setText(gps_text)
        except Exception as e:
            pass

    def connect_rocket_serial(self):
        if not self.running:
            selected_port = self.rocket_port_combo.currentText().split(' - ')[0]
            if not selected_port:
                return
            try:
                self.serial_port = serial.Serial(selected_port, 115200, timeout=1)
                self.running = True
                self.rocket_connect_btn.setText("🔌 Rocket Bağlantıyı Kes")
                threading.Thread(target=self.read_serial, daemon=True).start()
            except Exception as e:
                pass
        else:
            # Bağlantıyı kes
            self.running = False
            if self.serial_port:
                self.serial_port.close()
            self.rocket_connect_btn.setText("Roket Portuna Bağlan")

    def read_serial(self):
        """Read serial data using ring buffer for packet synchronization"""
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    # Read available data
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    if data:
                        # Add data to ring buffer
                        self.add_to_ring_buffer(data)
                        # Process complete packets from ring buffer
                        self.process_ring_buffer_packets()
            except Exception as e:
                self.data_buffer.append(f"Hata okuma sırasında: {e}")
                self.running = False
                
    def add_to_ring_buffer(self, data):
        """Add incoming data to ring buffer"""
        for byte in data:
            # Add byte to ring buffer
            self.ring_buffer[self.ring_buffer_head] = byte
            self.ring_buffer_head = (self.ring_buffer_head + 1) % len(self.ring_buffer)
            
            # Update buffer size
            if self.ring_buffer_size < len(self.ring_buffer):
                self.ring_buffer_size += 1
            else:
                # Buffer is full, move tail
                self.ring_buffer_tail = (self.ring_buffer_tail + 1) % len(self.ring_buffer)
                
    def process_ring_buffer_packets(self):
        """Process complete packets from ring buffer"""
        PACKET_SIZE = 43
        HEADER = 0xAA
        FOOTER = 0xFF
        
        while self.ring_buffer_size >= PACKET_SIZE:
            # Search for packet header
            header_found = False
            search_start = self.ring_buffer_tail
            
            for i in range(self.ring_buffer_size - PACKET_SIZE + 1):
                current_pos = (search_start + i) % len(self.ring_buffer)
                
                if self.ring_buffer[current_pos] == HEADER:
                    # Check if we have enough data for a complete packet
                    if self.ring_buffer_size >= i + PACKET_SIZE:
                        # Check footer
                        footer_pos = (current_pos + PACKET_SIZE - 1) % len(self.ring_buffer)
                        if self.ring_buffer[footer_pos] == FOOTER:
                            # Extract complete packet
                            packet = bytearray(PACKET_SIZE)
                            for j in range(PACKET_SIZE):
                                packet[j] = self.ring_buffer[(current_pos + j) % len(self.ring_buffer)]
                            
                            # Parse the packet
                            self.parse_binary_packet(packet)
                            
                            # Remove processed data from buffer
                            self.remove_from_ring_buffer(i + PACKET_SIZE)
                            header_found = True
                            break
                    else:
                        # Not enough data for complete packet
                        break
                        
            if not header_found:
                # No valid packet found, remove one byte and continue
                self.remove_from_ring_buffer(1)
                
    def remove_from_ring_buffer(self, count):
        """Remove specified number of bytes from ring buffer"""
        if count > self.ring_buffer_size:
            count = self.ring_buffer_size
            
        self.ring_buffer_tail = (self.ring_buffer_tail + count) % len(self.ring_buffer)
        self.ring_buffer_size -= count

    def parse_angles(self, line):
        """Legacy function - now handled by binary packet parsing"""
        # This function is kept for compatibility but binary packets are handled in parse_binary_packet
        pass
    
    def parse_binary_packet(self, data):
        """Binary packet formatını parse et (39 bytes total)"""
        try:
            # Packet header and footer already validated by ring buffer
            # data[0] should be 0xAA and data[38] should be 0xFF
            
            # Parse float32 values (big-endian format)
            self.altitude = struct.unpack('>f', data[1:5])[0]  # Bytes 1-4
            self.latitude = struct.unpack('>f', data[5:9])[0]  # Bytes 5-8
            self.longitude = struct.unpack('>f', data[9:13])[0]  # Bytes 9-12
            
            # Parse accelerometer data
            self.accel_x = struct.unpack('>f', data[13:17])[0]  # Bytes 13-16
            self.accel_y = struct.unpack('>f', data[17:21])[0]  # Bytes 17-20
            self.accel_z = struct.unpack('>f', data[21:25])[0]  # Bytes 21-24
            
            # Parse orientation data
            self.roll = struct.unpack('>f', data[25:29])[0]  # Bytes 25-28
            self.pitch = struct.unpack('>f', data[29:33])[0]  # Bytes 29-32
            self.yaw = struct.unpack('>f', data[33:37])[0]  # Bytes 33-36

            self.gpsaltitude = struct.unpack('>f', data[37:41])[0]  # Bytes 37-40
            
            # Parse status byte
            self.status_byte = data[41]  # Byte 41

            self.angle_z = self.calculate_z_angle_from_accel(
                self.accel_x, self.accel_y, self.accel_z
                )
            
            # Update gyroscope data (using orientation derivatives)
            # This is a simplified approach - in real implementation you'd get actual gyro data
            self.gyro_x = 0.0  # Will be implemented later
            self.gyro_y = 0.0
            self.gyro_z = 0.0
            
            self.data_buffer.append(f"Packet parsed: Alt={self.altitude:.1f}m, Lat={self.latitude:.6f}, Lon={self.longitude:.6f}")
            self.data_buffer.append(f"Orientation: Yaw={self.yaw:.1f}°, Pitch={self.pitch:.1f}°, Roll={self.roll:.1f}°")
            self.data_buffer.append(f"Accel: X={self.accel_x:.2f}, Y={self.accel_y:.2f}, Z={self.accel_z:.2f}")
            self.data_buffer.append(f"Status: 0x{status_byte:02X}")
            
        except Exception as e:
            self.data_buffer.append(f"Binary packet parse error: {e}")
            pass

    def map_status_byte(self, status):
        mapping = {
            0b00000000: 1,
            0b10000000: 1,
            0b11000000: 1,
            0b11100000: 1,
            0b11110000: 1,
            0b11111000: 1,
            0b11111100: 2,
            0b11111110: 2,
            0b11111111: 4,
        }
        return mapping.get(status, 3)


    def add_to_payload_ring_buffer(self, data):
        for byte in data:
            self.payload_ring_buffer[self.payload_ring_head] = byte
            self.payload_ring_head = (self.payload_ring_head + 1) % len(self.payload_ring_buffer)

            if self.payload_ring_size < len(self.payload_ring_buffer):
                self.payload_ring_size += 1
            else:
                self.payload_ring_tail = (self.payload_ring_tail + 1) % len(self.payload_ring_buffer)

    def process_payload_ring_buffer_packets(self):
        PACKET_SIZE = 34
        HEADER = 0xAA
        FOOTER = 0xFF

        while self.payload_ring_size >= PACKET_SIZE:
            header_found = False
            search_start = self.payload_ring_tail

            for i in range(self.payload_ring_size - PACKET_SIZE + 1):
                current_pos = (search_start + i) % len(self.payload_ring_buffer)

                if self.payload_ring_buffer[current_pos] == HEADER:
                    if self.payload_ring_size >= i + PACKET_SIZE:
                        footer_pos = (current_pos + PACKET_SIZE - 1) % len(self.payload_ring_buffer)
                        if self.payload_ring_buffer[footer_pos] == FOOTER:
                            packet = bytearray(PACKET_SIZE)
                            for j in range(PACKET_SIZE):
                                packet[j] = self.payload_ring_buffer[(current_pos + j) % len(self.payload_ring_buffer)]
                            
                            self.parse_payload_packet(packet)
                            self.remove_from_payload_ring_buffer(i + PACKET_SIZE)
                            header_found = True
                            break
                    else:
                        break

            if not header_found:
                self.remove_from_payload_ring_buffer(1)
    def parse_payload_packet(self, data):
        try:
            # data[0] = 0xAA, data[33] = 0xFF

            self.payload_latitude = struct.unpack('>f', data[1:5])[0]   # Bytes 1-4

            self.payload_longitude = struct.unpack('>f', data[5:9])[0]  # Bytes 5-8

            self.payload_altitude = struct.unpack('>f', data[9:13])[0]  # Bytes 9-12
            
            self.payload_strain = struct.unpack('>f', data[13:17])[0]  # Bytes 13-16

            self.payload_bmealtitude = struct.unpack('>f', data[17:21])[0]  # Bytes 17-20

            self.payload_temperature = struct.unpack('>f', data[21:25])[0]  # Bytes 21-24

            self.payload_pressure = struct.unpack('>f', data[25:29])[0]  # Bytes 25-28

            self.payload_humidity = struct.unpack('>f', data[29:33])[0]  # Bytes 29-32

            self.payload_data_buffer.append(
                f"Payload parsed: Lat={self.payload_latitude:.6f}, Lon={self.payload_longitude:.6f}, Alt={self.payload_altitude:.1f}m"
            )

        except Exception as e:
            self.payload_data_buffer.append(f"Payload parse error: {e}")


    def remove_from_payload_ring_buffer(self, count):
        if count > self.payload_ring_size:
            count = self.payload_ring_size
        self.payload_ring_tail = (self.payload_ring_tail + count) % len(self.payload_ring_buffer)
        self.payload_ring_size -= count


    def update_rocket_display(self):
        """Roket görselini güncelle"""
        self.rocket_widget.set_angles(self.yaw, self.pitch, self.roll)

    def update_gui(self):
        """GUI'yi güncelle"""
        # Clear data buffers (no longer displaying in textbox)
        while self.data_buffer:
            self.data_buffer.pop(0)
        
        # Clear payload data buffers (no longer displaying in textbox)
        while self.payload_data_buffer:
            self.payload_data_buffer.pop(0)
        
        # Açı değerlerini güncelle
        self.angle_label.setText(f"""Açı Değerleri:
Yaw: {self.yaw:.1f}° 
Pitch: {self.pitch:.1f}° 
Roll: {self.roll:.1f}° 
Z-Açı : {self.angle_z:.1f}° 
Accel: X={self.accel_x:.2f} Y={self.accel_y:.2f} Z={self.accel_z:.2f}
Status: 0x{self.status_byte:02X}""")
        
        # İrtifa değerini güncelle
        self.altitude_label.setText(f"İrtifa:\n{self.altitude:.1f} m\nGPS İrtifa:\n{self.gpsaltitude:.1f} m")

        
        # GPS koordinatlarını güncelle
        self.gps_label.setText(f"""GPS Koordinatları:
Enlem: {self.latitude:.6f}° 
Boylam: {self.longitude:.6f}° """)
        
        # Görev Yükü GPS koordinatlarını güncelle
        self.payload_gps_label.setText(f"""Görev Yükü:
Enlem: {self.payload_latitude:.6f}° 
Boylam: {self.payload_longitude:.6f}° 
İrtifa: {self.payload_altitude:.1f} m
Strain: {self.payload_strain:.2f}
İrtifa Basınç: {self.payload_bmealtitude:.2f}
Basınç: {self.payload_pressure:.2f}
Sıcaklık: {self.payload_temperature:.2f}
Nem: {self.payload_humidity:.2f}""")
    

        
        # Roket görselini güncelle
        self.update_rocket_display()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = YerIstasyonu()
    window.show()
    sys.exit(app.exec())