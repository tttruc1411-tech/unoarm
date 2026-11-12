# kdi_unoarm.py  --  UnoArm v3, ban cho board ORC (Yolo UNO + Motor Driver V2)
#
# FILE NAY LA NOI DUY NHAT chua giai thuat va goi thu vien phan cung (mdv2, servo, constants).
# main.py chi GOI cac ham cua file nay (day cung la code ma khoi block se sinh ra).
#
# Hai nhom ham:
#   - Ham tinh toan, goi binh thuong (dung duoc nhu khoi gia tri):
#       kiem_tra_tinh_hop_le(S2, S3), tinh_goc(r, h), tinh_vi_tri(S2, S3), kiem_tra_duong_di(S2, S3),
#       co_the_toi(r, h), set_dimensions(d1..d5), khoi_tao(...), dat_gioi_han(...), dat_toc_do(...),
#       dat_toc_do_phan_tram(...), thong_tin(),
#       dung_lai() (dung ngay lenh di chuyen dang chay; dang ky trong hook yolo_uno.deinit)
#   - Bo khoi chi co 8 khoi (nhu ban xbot), KHONG co khoi khoi tao va KHONG co nut BOOT: keo khoi di chuyen ra la chay.
#     Lan di chuyen dau tien thu vien tu khoi tao ngay (noi servo va gui tu the bat dau (10, 10), xem _dam_bao_khoi_tao).
#     Cac ham khac (dat_gioi_han, dat_toc_do_phan_tram, co_the_toi, quay_ve_goc, ve_vi_tri_dau, dung_lai,
#     cho_nhan_BOOT) van co de goi bang code.
#   - Ham di chuyen, la `async`, PHAI goi bang `await` (khong chan cac task khac nhu task_forever, BLE):
#       await go_to_S2_S3(S2, S3), await go_to_r_h(r, h), await quay_ve_goc(goc_S1), await ve_vi_tri_dau()
#   Mot luc chi co mot lenh di chuyen S2/S3 chay; lenh thu hai goi khi lenh truoc chua xong se bi tu choi.
#
# AN TOAN (doc ky truoc khi chay that):
#  1. khoi_tao(dry_run=True) (mac dinh): chi tinh toan va in log, KHONG gui lenh nao toi servo.
#  2. Servo cua board ORC khong doc nguoc duoc goc. Code tu nho S2_now, S3_now.
#     Phai DAT TAY robot dung tu the bat dau (S2, S3) = (10, 10) truoc khi cap nguon servo,
#     roi moi goi khoi_tao(dry_run=False) (ham nay gui tu the bat dau de servo giu o do).
#  3. Moi lan di chuyen: tinh san CA DUONG DI (moi buoc 1 do), kiem tra tung diem nam trong hinh thang
#     va khong thap hon H_TOI_THIEU. Neu co diem xau thi KHONG gui gi ca.
#  4. Hinh thang la vung loi, nen duong thang giua hai diem hop le van hop le.
#
# Vung hop le (S2, S3), hinh thang voi 4 dinh (10,10) (75,10) (140,100) (10,100):
#    S2 >= 10,  10 <= S3 <= 100,  18*S2 - 13*S3 <= 1220   (canh xien qua (75,10) va (140,100))
#
# Cong thuc v3 (H2 = 90, H3 = 70, sigma2 = -1, sigma3 = +1, banh rang 2:1 ca hai truc):
#    phi1 = 185 - S2/2            phi2 = (S2 + S3)/2 + 20
#    S2 = 370 - 2*phi1            S3 = 2*(phi1 + phi2) - 410

import math
import asyncio

# Thu vien nen: extension di kem servo.py, mdv2.py, constants.py la ban sao GIONG HET TUNG BYTE cua
# yolouno_extension_robotics (de chua add robotics van dieu khien duoc canh tay; add robotics thi file trung bi ghi de
# ma khong doi gi). File nay chi can:
#   mdv2.MotorDriverV2()                 (khoi tao khong doi so, co set_servo(index, angle, max))
#   servo.Servo(driver, cong, 180)       (co .limit(min, max) va .angle(goc))
#   constants.S1, S2, S3                 (neu thieu thi dung chi so 0, 1, 2 cua Motor Driver V2)
# Nen cung chay duoc voi ban cua board va cua visionbot_espcam.
try:
    from mdv2 import MotorDriverV2
    from servo import Servo
    CO_PHAN_CUNG = True
except ImportError:
    CO_PHAN_CUNG = False                # may tinh, hoac board thieu thu vien nen (xem _dam_bao_khoi_tao)
try:
    from constants import S1 as PORT_S1, S2 as PORT_S2, S3 as PORT_S3
except ImportError:
    PORT_S1, PORT_S2, PORT_S3 = 0, 1, 2
PORT_M1 = 1     # bitmask cong dong co M1 cua Motor Driver V2 (co dinh, khong phu thuoc constants.py)

# ---------------------------------------------------------------------------------------------
# KICH THUOC (cm)
# ---------------------------------------------------------------------------------------------
_d1 = 10.54   # OA: cot de, tu mat ban len vai A
_d2 = 15      # AB: truc lon (S2)
_d3 = 16      # BC: truc dau (S3)
_d4 = 4.64    # CD: doan ngang
_d5 = 5       # DE: doan dung toi dau hut

# ---------------------------------------------------------------------------------------------
# VUNG HOP LE (hinh thang)
# ---------------------------------------------------------------------------------------------
S1_MIN = 0
S1_MAX = 180
S2_MIN = 10
S3_MIN = 10
S3_MAX = 100
CX_A = 18      # canh xien: CX_A*S2 - CX_B*S3 <= CX_C
CX_B = 13
CX_C = 1220
S2_MAX = (CX_C + CX_B * S3_MAX) // CX_A   # = 140, S2 lon nhat (tai S3 = 100)

# ---------------------------------------------------------------------------------------------
# TU THE BAT DAU: dat tay robot o day truoc khi cap nguon
# ---------------------------------------------------------------------------------------------
S1_BAT_DAU = 90
S2_BAT_DAU = 10
S3_BAT_DAU = 10

# ---------------------------------------------------------------------------------------------
# CAI DAT (co the doi tu main)
# ---------------------------------------------------------------------------------------------
DRY_RUN = True          # True: khong gui lenh servo (dat boi khoi_tao)
LOG = True              # in log
LOG_TUNG_BUOC = False   # in tung buoc 1 do khi di chuyen
DELAY_MS = 30           # ms moi buoc 1 do (30 ms ~ 33 do/giay)
TOI_DA_MOI_LAN = None   # do toi da cho moi lan di chuyen (None = khong gioi han)
H_TOI_THIEU = None      # cm, dau hut khong duoc thap hon muc nay doc duong di (None = khong kiem)

# ---------------------------------------------------------------------------------------------
# TRANG THAI
# ---------------------------------------------------------------------------------------------
S1_now = S1_BAT_DAU
S2_now = S2_BAT_DAU
S3_now = S3_BAT_DAU
r_now = 0
h_now = 0
phi_now = 90

_md = None
servo1 = None
servo2 = None
servo3 = None
_dang_chay = False      # dang di chuyen S2/S3
_dang_chay_S1 = False   # dang quay S1
_trang_thai_dau_hut = False # trang thai bat/tat cua dau hut M1
_yeu_cau_dung = False   # dung_lai() dat True de cac vong di chuyen dang chay thoat ngay
_da_khoi_tao = False    # khoi_tao() da chay xong (goi tay hoac tu dong o lan di chuyen dau tien)


# ---------------------------------------------------------------------------------------------
# HAM PHU
# ---------------------------------------------------------------------------------------------
def _log(*args):
    if LOG:
        print(*args)


async def _asleep_ms(ms):
    try:
        sleep = asyncio.sleep_ms
    except AttributeError:
        await asyncio.sleep(ms / 1000)
        return
    await sleep(ms)


def _clamp1(v):
    if v > 1:
        return 1
    if v < -1:
        return -1
    return v


def set_dimensions(d1, d2, d3, d4, d5):
    global _d1, _d2, _d3, _d4, _d5
    _d1 = d1
    _d2 = d2
    _d3 = d3
    _d4 = d4
    _d5 = d5
    _cap_nhat_vi_tri()


# ---------------------------------------------------------------------------------------------
# HINH HOC
# ---------------------------------------------------------------------------------------------
def goc_tu_servo(S2, S3):
    """(S2, S3) -> (phi1, phi2) tinh bang do."""
    return 185 - S2 / 2, (S2 + S3) / 2 + 20


def vi_tri_tu_goc(phi1, phi2):
    """(phi1, phi2) -> (r, h) cua dau hut E."""
    a = math.radians(phi1 - 90)
    by = _d2 * math.cos(a)
    bz = _d1 + _d2 * math.sin(a)
    g = math.radians(phi1 + phi2 - 270)
    cy = by + _d3 * math.cos(g)
    cz = bz + _d3 * math.sin(g)
    return cy + _d4, cz - _d5


def tinh_vi_tri(S2, S3):
    """(S2, S3) -> (r, h) cua dau hut E (hinh hoc thuan)."""
    phi1, phi2 = goc_tu_servo(S2, S3)
    return vi_tri_tu_goc(phi1, phi2)


def tinh_goc(r, h):
    """(r, h) -> goc servo (hinh hoc nguoc). Tra ve dict, k['ok'] la True neu voi toi duoc.
    Chua kiem tra hinh thang: viec do la cua kiem_tra_tinh_hop_le."""
    k = {'ok': False, 'ly_do': ''}
    y1 = r - _d4
    z1 = h + _d5
    d6 = math.sqrt(y1 * y1 + (z1 - _d1) * (z1 - _d1))
    k['y1'] = y1
    k['z1'] = z1
    k['d6'] = d6
    d6_min = abs(_d2 - _d3)
    d6_max = _d2 + _d3
    if d6 < 1e-9 or d6 < d6_min - 1e-9 or d6 > d6_max + 1e-9:
        k['ly_do'] = 'ngoai tam voi: d6 = %.2f, cho phep tu %.2f den %.2f' % (d6, d6_min, d6_max)
        return k
    phi2 = math.degrees(math.acos(_clamp1((_d2 * _d2 + _d3 * _d3 - d6 * d6) / (2 * _d2 * _d3))))
    phi3 = math.degrees(math.acos(_clamp1((_d2 * _d2 + d6 * d6 - _d3 * _d3) / (2 * _d2 * d6))))
    s = math.degrees(math.asin(_clamp1(y1 / d6)))
    if z1 > _d1:
        phi4 = 180 - s
    else:
        phi4 = s
    phi1 = phi3 + phi4
    k['phi1'] = phi1
    k['phi2'] = phi2
    k['phi3'] = phi3
    k['phi4'] = phi4
    k['S2'] = 370 - 2 * phi1
    k['S3'] = 2 * (phi1 + phi2) - 410
    k['ok'] = True
    return k


def _cap_nhat_vi_tri():
    global r_now, h_now
    r_now, h_now = tinh_vi_tri(S2_now, S3_now)


# ---------------------------------------------------------------------------------------------
# TINH HOP LE (hinh thang)
# ---------------------------------------------------------------------------------------------
def ly_do_khong_hop_le(S2, S3):
    """Tra ve danh sach cac dieu kien bi vi pham (rong neu hop le). Khong in gi."""
    if S2 != S2 or S3 != S3:
        return ['S2 hoac S3 khong phai so']
    loi = []
    if S2 < S2_MIN:
        loi.append('S2 = %.2f < %d' % (S2, S2_MIN))
    if S3 < S3_MIN:
        loi.append('S3 = %.2f < %d' % (S3, S3_MIN))
    if S3 > S3_MAX:
        loi.append('S3 = %.2f > %d' % (S3, S3_MAX))
    v = CX_A * S2 - CX_B * S3
    if v > CX_C + 1e-9:
        loi.append('%d*S2 - %d*S3 = %.2f > %d (ra ngoai canh xien)' % (CX_A, CX_B, v, CX_C))
    return loi


def kiem_tra_tinh_hop_le(check_S2, check_S3):
    loi = ly_do_khong_hop_le(check_S2, check_S3)
    hop_le = len(loi) == 0
    if hop_le:
        _log('Kiem tra hop le (S2 = %s, S3 = %s): True' % (check_S2, check_S3))
    else:
        _log('Kiem tra hop le (S2 = %s, S3 = %s): False <- %s' % (check_S2, check_S3, '; '.join(loi)))
    return hop_le


# ---------------------------------------------------------------------------------------------
# DUONG DI: tinh san, kiem tra het roi moi gui
# ---------------------------------------------------------------------------------------------
def _lap_duong_di(ts2, ts3):
    """Lap duong di tu (S2_now, S3_now) toi (ts2, ts3), moi buoc toi da 1 do.
    Tra ve (ok, path, h_min, thong_bao). Khong gui lenh, khong doi trang thai."""
    loi = ly_do_khong_hop_le(S2_now, S3_now)
    if loi:
        return False, [], None, 'diem hien tai (%s, %s) khong hop le: %s' % (S2_now, S3_now, '; '.join(loi))
    loi = ly_do_khong_hop_le(ts2, ts3)
    if loi:
        return False, [], None, 'diem dich (%s, %s) khong hop le: %s' % (ts2, ts3, '; '.join(loi))
    d2 = ts2 - S2_now
    d3 = ts3 - S3_now
    n = max(abs(d2), abs(d3))
    if n == 0:
        return True, [], None, 'da o dung diem dich'
    if TOI_DA_MOI_LAN is not None and n > TOI_DA_MOI_LAN:
        return False, [], None, 'buoc di %d do vuot gioi han moi lan %s do (doi TOI_DA_MOI_LAN neu muon)' % (n, TOI_DA_MOI_LAN)
    path = []
    h_min = None
    pa = S2_now
    pb = S3_now
    for i in range(1, n + 1):
        x = S2_now + d2 * i / n
        y = S3_now + d3 * i / n
        if i == n:
            cands = [(ts2, ts3)]
        else:
            # servo chi nhan so nguyen: thu lam tron gan nhat truoc, roi cac o nguyen quanh diem ly tuong
            fx = int(math.floor(x))
            fy = int(math.floor(y))
            cands = [(int(round(x)), int(round(y))), (fx, fy), (fx, fy + 1), (fx + 1, fy), (fx + 1, fy + 1)]
        chon = None
        loi_cuoi = []
        for (a, b) in cands:
            if abs(a - pa) > 1 or abs(b - pb) > 1:
                continue
            loi = ly_do_khong_hop_le(a, b)
            if loi:
                loi_cuoi = loi
                continue
            chon = (a, b)
            break
        if chon is None:
            return False, [], None, 'khong co diem nguyen hop le gan (%.1f, %.1f) tren duong di: %s' % (x, y, '; '.join(loi_cuoi))
        pa, pb = chon
        h = tinh_vi_tri(pa, pb)[1]
        if h_min is None or h < h_min:
            h_min = h
        path.append(chon)
    if H_TOI_THIEU is not None and h_min < H_TOI_THIEU - 1e-9:
        return False, [], h_min, 'dau hut ha xuong h = %.2f cm, thap hon H_TOI_THIEU = %s cm' % (h_min, H_TOI_THIEU)
    return True, path, h_min, 'ok'


def kiem_tra_duong_di(to_S2, to_S3):
    """Chi kiem tra va in log duong di toi (to_S2, to_S3), KHONG di chuyen. Tra ve True neu di duoc."""
    ts2 = int(round(to_S2))
    ts3 = int(round(to_S3))
    ok, path, h_min, msg = _lap_duong_di(ts2, ts3)
    _log('Duong di (%d, %d) -> (%d, %d): %s' % (S2_now, S3_now, ts2, ts3, 'DI DUOC' if ok else 'KHONG DI'))
    if not ok:
        _log('  ly do: ' + msg)
        return False
    if path:
        r2, h2 = tinh_vi_tri(ts2, ts3)
        _log('  %d buoc 1 do, ~%d ms, h thap nhat doc duong = %.2f cm, dich r = %.2f, h = %.2f'
             % (len(path), len(path) * DELAY_MS, h_min, r2, h2))
    else:
        _log('  ' + msg)
    return True


# ---------------------------------------------------------------------------------------------
# DI CHUYEN (async: goi bang await)
# ---------------------------------------------------------------------------------------------
def dung_lai():
    """Dung NGAY lenh di chuyen dang chay (servo giu o goc da gui toi buoc cuoi). Khong lam gi neu dang dung yen.
    Dung cho khoi 'dung lai' va cho hook yolo_uno.deinit khi chuong trinh bi dung."""
    global _yeu_cau_dung, _trang_thai_dau_hut
    if _dang_chay or _dang_chay_S1:
        _yeu_cau_dung = True
        print('dung_lai: dung lenh dang chay')
    if _md is not None and not DRY_RUN:
        try:
            _md.stop(PORT_M1)
        except Exception:
            pass
    _trang_thai_dau_hut = False


def _het_yeu_cau_dung():
    # goi khi mot vong di chuyen ket thuc; vong cuoi cung ket thuc se xoa yeu cau dung
    global _yeu_cau_dung
    if not _dang_chay and not _dang_chay_S1:
        _yeu_cau_dung = False


def _san_sang():
    if not DRY_RUN and servo2 is None:
        print('chua khoi_tao(dry_run=False): chua noi servo')
        return False
    return True


async def _dam_bao_khoi_tao():
    """Khong co khoi 'khoi tao' trong bo khoi, nen lan di chuyen dau tien tu khoi tao neu chua khoi_tao(), NGAY (khong
    cho nut BOOT, giong ban xbot): tren board noi servo va gui tu the bat dau (10, 10) (robot phai duoc dat tay o do
    truoc khi cap nguon, neu khong servo se nhay ve (10, 10)). Tren may tinh (khong co thu vien phan cung): chay thu
    (dry_run). Tra ve True neu san sang. Khong co await ben trong nen hai lenh goi chong khong the khoi tao doi."""
    if _da_khoi_tao:
        return True
    if CO_PHAN_CUNG:
        return khoi_tao(dry_run=False)
    if _la_board():
        # dang tren board nhung thieu mdv2/servo: KHONG am tham chay thu, bao ro de nguoi dung them thu vien nen
        print('LOI: board thieu thu vien mdv2.py / servo.py. Hay them extension robotics goc (yolouno_extension_robotics) '
              'roi chay lai. Robot khong nhan lenh nao.')
        return False
    return khoi_tao(dry_run=True)


async def go_to_S2_S3(to_S2, to_S3, delay_ms=None):
    global S2_now, S3_now, _dang_chay
    if delay_ms is None:
        delay_ms = DELAY_MS
    if not await _dam_bao_khoi_tao():
        return False
    _log('di den goc: %s , %s' % (to_S2, to_S3))
    if not _san_sang():
        return False
    if _dang_chay:
        print('dang di chuyen, bo qua lenh moi')
        return False
    ts2 = int(round(to_S2))
    ts3 = int(round(to_S3))
    ok, path, h_min, msg = _lap_duong_di(ts2, ts3)
    if not ok:
        print('khong hop le: ' + msg)
        return False
    if not path:
        _log(msg)
        return True
    _log('%s: %d buoc, h thap nhat = %.2f cm' % ('[DRY-RUN] mo phong' if DRY_RUN else 'gui lenh servo', len(path), h_min))
    _dang_chay = True
    da_dung = False
    try:
        for (a, b) in path:
            if _yeu_cau_dung:
                da_dung = True
                break
            if not DRY_RUN:
                servo2.angle(a)
                servo3.angle(b)
            S2_now = a              # cap nhat ngay sau khi gui, de neu lenh bi huy giua duong thi S2_now, S3_now van dung
            S3_now = b
            if LOG_TUNG_BUOC:
                print('  S2 = %d, S3 = %d' % (a, b))
            if not DRY_RUN:
                await _asleep_ms(delay_ms)
    finally:
        _dang_chay = False
        _het_yeu_cau_dung()
        _cap_nhat_vi_tri()
    if da_dung:
        print('da dung giua duong tai S2 = %d, S3 = %d (r = %.2f, h = %.2f)' % (S2_now, S3_now, r_now, h_now))
        return False
    _log('S2 va S3 moi duoc cap nhat: %d , %d  (r = %.2f, h = %.2f)' % (S2_now, S3_now, r_now, h_now))
    return True


async def go_to_r_h(r, h, delay_ms=None):
    if not await _dam_bao_khoi_tao():
        return False
    _log('-----------------')
    _log('di den vi tri: r = %s , h = %s' % (r, h))
    k = tinh_goc(r, h)
    if not k['ok']:
        print('khong hop le: ' + k['ly_do'])
        return False
    _log('y1 = %.3f, z1 = %.3f, d6 = %.3f' % (k['y1'], k['z1'], k['d6']))
    _log('phi1 = %.3f, phi2 = %.3f, phi3 = %.3f, phi4 = %.3f' % (k['phi1'], k['phi2'], k['phi3'], k['phi4']))
    _log('to_S2 = %.3f, to_S3 = %.3f' % (k['S2'], k['S3']))
    ok = await go_to_S2_S3(k['S2'], k['S3'], delay_ms)
    if ok:
        _log('vi tri that (theo goc da lam tron): r = %.2f, h = %.2f; sai lech so voi muc tieu: dr = %.2f, dh = %.2f'
             % (r_now, h_now, r_now - r, h_now - h))
        _log('r_now = %.2f , h_now = %.2f' % (r_now, h_now))
    return ok


async def quay_ve_goc(goc_S1, delay_ms=None):
    """Quay de S1 (90 = chinh giua). Gioi han [0, 180]. Neu vuot nguong se tu dong ve bien va dung yen."""
    global S1_now, _dang_chay_S1
    if delay_ms is None:
        delay_ms = DELAY_MS
    if not await _dam_bao_khoi_tao():
        return False
    if not _san_sang():
        return False
    if _dang_chay_S1:
        print('S1 dang quay, bo qua lenh moi')
        return False
    try:
        g = int(round(goc_S1))
    except (TypeError, ValueError):
        print('khong hop le: goc S1 khong phai so:', goc_S1)
        return False
    if g < S1_MIN:
        _log('S1 = %d nho hon %d, gioi han ve %d' % (g, S1_MIN, S1_MIN))
        g = S1_MIN
    elif g > S1_MAX:
        _log('S1 = %d lon hon %d, gioi han ve %d' % (g, S1_MAX, S1_MAX))
        g = S1_MAX
    if S1_now == g:
        _log('S1 da o goc %d' % g)
        return True
    n = abs(g - S1_now)
    if TOI_DA_MOI_LAN is not None and n > TOI_DA_MOI_LAN:
        print('khong hop le: buoc quay S1 %d do vuot gioi han moi lan %s do' % (n, TOI_DA_MOI_LAN))
        return False
    step = 1 if g > S1_now else -1
    _dang_chay_S1 = True
    da_dung = False
    try:
        while S1_now != g:
            if _yeu_cau_dung:
                da_dung = True
                break
            S1_now += step
            if not DRY_RUN:
                servo1.angle(S1_now)
                await _asleep_ms(delay_ms)
    finally:
        _dang_chay_S1 = False
        _het_yeu_cau_dung()
    if da_dung:
        print('da dung giua duong, S1 = %d' % S1_now)
        return False
    _log('S1 = %d' % S1_now)
    return True


async def ve_vi_tri_dau():
    """Ve tu the bat dau an toan (S2, S3) = (10, 10)."""
    return await go_to_S2_S3(S2_BAT_DAU, S3_BAT_DAU)


# ---------------------------------------------------------------------------------------------
# DAU HUT CHAN KHONG (CONG DONG CO M1)
# ---------------------------------------------------------------------------------------------
def _dieu_khien_dau_hut_core(bat, toc_do=100):
    global _trang_thai_dau_hut
    _trang_thai_dau_hut = bool(bat)
    if DRY_RUN:
        print('[DRY-RUN] Dau hut (M1): %s (toc do %d%%)' % ('BAT' if bat else 'TAT', toc_do if bat else 0))
        return True
    if _md is not None:
        try:
            if bat:
                _md.set_motors(PORT_M1, int(toc_do))
            else:
                _md.stop(PORT_M1)
        except Exception as e:
            print('Loi dieu khien dau hut M1:', e)
            return False
    _log('Dau hut (M1): %s' % ('BAT' if bat else 'TAT'))
    return True


async def dieu_khien_dau_hut(bat=True, toc_do=100):
    """Bat hoac tat dau hut chan khong o cong M1.
    bat=True/1: bat dau hut (toc do 100%)
    bat=False/0: tat dau hut (dung dong co M1, nha vat)"""
    if not await _dam_bao_khoi_tao():
        return False
    return _dieu_khien_dau_hut_core(bat, toc_do)


async def hut(toc_do=100):
    """Bat dau hut o cong M1."""
    return await dieu_khien_dau_hut(True, toc_do)


async def tha():
    """Tat dau hut o cong M1."""
    return await dieu_khien_dau_hut(False, 0)


def bat_dau_hut(toc_do=100):
    """Ham dong bo: bat dau hut ngay neu driver da san sang."""
    return _dieu_khien_dau_hut_core(True, toc_do)


def tat_dau_hut():
    """Ham dong bo: tat dau hut ngay."""
    return _dieu_khien_dau_hut_core(False, 0)


def trang_thai_dau_hut():
    """Tra ve True neu dau hut dang bat, False neu dang tat."""
    return _trang_thai_dau_hut


# ---------------------------------------------------------------------------------------------
# KHOI TAO VA THONG TIN
# ---------------------------------------------------------------------------------------------
def _tim_driver():
    """Dung lai Motor Driver V2 ma chuong trinh da tao san (bien md_v2 cua khung main do cac khoi robotics goc sinh ra),
    de hai thu vien dung chung mot driver thay vi tao hai cai (hai lan khoi tao SoftI2C va PWM tren cung chan)."""
    try:
        import sys
        md = getattr(sys.modules.get('__main__'), 'md_v2', None)
        if md is not None and isinstance(md, MotorDriverV2):
            return md
    except Exception:
        pass
    return MotorDriverV2()


def _la_board():
    try:
        import machine
        return True
    except ImportError:
        return False


def khoi_tao(dry_run=True, S2_ban_dau=S2_BAT_DAU, S3_ban_dau=S3_BAT_DAU):
    """dry_run=True: chi tinh toan. dry_run=False: tao MotorDriverV2 va 3 servo (S1, S2, S3), roi GUI tu the
    bat dau toi S2, S3 (robot phai da duoc dat tay dung o tu the do, de S1 o 90)."""
    global DRY_RUN, _md, servo1, servo2, servo3, S1_now, S2_now, S3_now, _yeu_cau_dung, _da_khoi_tao
    loi = ly_do_khong_hop_le(S2_ban_dau, S3_ban_dau)
    if loi:
        print('tu the bat dau khong hop le: ' + '; '.join(loi))
        return False
    if not dry_run and not CO_PHAN_CUNG:
        print('khong tim thay thu vien mdv2/servo/constants (dang chay tren may tinh?): chi dung duoc dry_run=True')
        return False
    DRY_RUN = dry_run
    _yeu_cau_dung = False
    S1_now = S1_BAT_DAU
    S2_now = S2_ban_dau
    S3_now = S3_ban_dau
    _cap_nhat_vi_tri()
    if dry_run:
        print('[DRY-RUN] khong gui lenh servo. Tu the gia lap: S2 = %s, S3 = %s' % (S2_now, S3_now))
        _da_khoi_tao = True
        return True
    if _md is None:
        _md = _tim_driver()             # bao loi neu khong tim thay driver
    servo1 = Servo(_md, PORT_S1, 180)
    servo2 = Servo(_md, PORT_S2, 180)
    servo3 = Servo(_md, PORT_S3, 180)
    servo2.limit(S2_MIN, S2_MAX)        # lop chan thu hai, ngoai hinh thang
    servo3.limit(S3_MIN, S3_MAX)
    servo2.angle(S2_now)                # giu servo o tu the bat dau (robot da duoc dat tay o day)
    servo3.angle(S3_now)
    try:                                # S1 la phan them: loi o day khong duoc lam hong S2/S3
        servo1.limit(S1_MIN, S1_MAX)
        servo1.angle(S1_now)
    except Exception as e:
        print('Canh bao: khong khoi tao duoc S1:', e)
    print('da noi phan cung. Gui tu the bat dau S1 = %s, S2 = %s, S3 = %s' % (S1_now, S2_now, S3_now))
    _da_khoi_tao = True
    return True


def dat_gioi_han(toi_da_moi_lan, h_toi_thieu):
    """Gioi han an toan moi lan di chuyen. Truyen None de bo gioi han tuong ung."""
    global TOI_DA_MOI_LAN, H_TOI_THIEU
    TOI_DA_MOI_LAN = toi_da_moi_lan
    H_TOI_THIEU = h_toi_thieu


def dat_toc_do(delay_ms):
    """ms cho moi buoc 1 do (so lon = cham). 30 ms ~ 33 do/giay."""
    global DELAY_MS
    DELAY_MS = int(delay_ms)


def dat_toc_do_phan_tram(phan_tram):
    """Toc do tu 1 den 100 %. 100 % ~ 10 ms moi do (~100 do/giay), 50 % ~ 55 ms, 1 % ~ 99 ms."""
    global DELAY_MS
    p = max(1, min(100, phan_tram))
    DELAY_MS = int(round(100 - 0.9 * p))


def co_the_toi(r, h):
    """True neu tu vi tri hien tai co the DI NGAY toi (r, h): voi toi duoc, goc lam tron nam trong hinh thang,
    duong di hop le (gom ca gioi han moi lan va do cao toi thieu). Khong in log, khong di chuyen."""
    k = tinh_goc(r, h)
    if not k['ok']:
        return False
    ok, path, h_min, msg = _lap_duong_di(int(round(k['S2'])), int(round(k['S3'])))
    return ok


def _chan_boot():
    from machine import Pin
    try:
        from yolo_uno import BOOT_PIN
    except ImportError:
        BOOT_PIN = 0
    return Pin(BOOT_PIN)    # khong doi cau hinh chan: chi doc muc (nhan = 0)


async def cho_nhan_BOOT():
    """TUY CHON, KHONG tu goi (bo khoi khong dung nut BOOT). Cho den khi nhan nut BOOT (roi tha nut); doc truc tiep chan
    BOOT, khong tao aButton de khong xung dot voi khoi 'nut BOOT' cua he thong. Ai muon an toan hon thi goi
    `await cho_nhan_BOOT()` truoc lenh di chuyen dau tien."""
    print('Dat tay robot o (S2, S3) = (%d, %d), S1 o giua (%d), roi nhan nut BOOT de tiep tuc' % (S2_BAT_DAU, S3_BAT_DAU, S1_BAT_DAU))
    nut = _chan_boot()
    while nut.value() != 0:
        await _asleep_ms(50)
    await _asleep_ms(30)                # chong rung
    while nut.value() == 0:
        await _asleep_ms(30)
    await _asleep_ms(200)


def thong_tin():
    phi1, phi2 = goc_tu_servo(S2_now, S3_now)
    print('--- trang thai ---')
    print('che do: %s' % ('DRY-RUN (khong gui lenh)' if DRY_RUN else 'THAT (gui lenh servo)'))
    print('S1 = %d, S2 = %d, S3 = %d' % (S1_now, S2_now, S3_now))
    print('phi1 = %.2f, phi2 = %.2f, r = %.2f, h = %.2f' % (phi1, phi2, r_now, h_now))
    print('dau hut M1: %s' % ('BAT' if _trang_thai_dau_hut else 'TAT'))
    print('gioi han moi lan = %s do, H_TOI_THIEU = %s cm, delay = %d ms/buoc' % (TOI_DA_MOI_LAN, H_TOI_THIEU, DELAY_MS))


_cap_nhat_vi_tri()
