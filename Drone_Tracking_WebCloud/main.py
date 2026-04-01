
import asyncio
import json
import math
import os
import secrets
from contextlib import asynccontextmanager
from datetime import datetime, timedelta, timezone
from typing import Optional

import jwt
from fastapi import (
    Depends,
    FastAPI,
    HTTPException,
    Query,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, RedirectResponse
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from passlib.context import CryptContext
from pydantic import BaseModel, Field
from sqlalchemy import (
    Column,
    DateTime,
    Float,
    ForeignKey,
    Integer,
    String,
    Text,
    func,
    select,
    delete,
)
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import DeclarativeBase, sessionmaker, relationship

# ═══════════════════════════════════════════════════════════════════════════════
# 配 置
# ═══════════════════════════════════════════════════════════════════════════════

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATABASE_URL = os.getenv("DATABASE_URL", f"sqlite+aiosqlite:///{os.path.join(BASE_DIR, 'drone_tracking.db')}")
JWT_SECRET = os.getenv("JWT_SECRET", secrets.token_urlsafe(48))
JWT_ALGORITHM = "HS256"
JWT_EXPIRE_HOURS = int(os.getenv("JWT_EXPIRE_HOURS", "24"))

MAX_RECORDS = 100_000
DATA_AGING_DAYS = 7
AGING_INTERVAL_SECONDS = 300

DEFAULT_ADMIN_USERNAME = os.getenv("ADMIN_USERNAME", "admin")
DEFAULT_ADMIN_PASSWORD = os.getenv("ADMIN_PASSWORD", "admin123")

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security_scheme = HTTPBearer()

# ═══════════════════════════════════════════════════════════════════════════════
# 数据库 ORM 模型
# ═══════════════════════════════════════════════════════════════════════════════

class Base(DeclarativeBase):
    pass


class Location(Base):
    __tablename__ = "locations"
    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String(128), unique=True, nullable=False, index=True)
    description = Column(Text, nullable=False, default="")
    created_at = Column(DateTime, default=lambda: datetime.now(timezone.utc))
    cameras = relationship("Camera", back_populates="location", cascade="all, delete-orphan")


class Camera(Base):
    __tablename__ = "cameras"
    id = Column(Integer, primary_key=True, autoincrement=True)
    camera_name = Column(String(64), unique=True, nullable=False, index=True)
    location_id = Column(Integer, ForeignKey("locations.id", ondelete="CASCADE"), nullable=False)
    status = Column(String(16), nullable=False, default="active")
    created_at = Column(DateTime, default=lambda: datetime.now(timezone.utc))
    location = relationship("Location", back_populates="cameras")


class User(Base):
    __tablename__ = "users"
    id = Column(Integer, primary_key=True, autoincrement=True)
    username = Column(String(64), unique=True, nullable=False, index=True)
    password_hash = Column(Text, nullable=False)
    role = Column(String(16), nullable=False, default="user")
    allowed_cameras = Column(Text, nullable=False, default='[]')
    created_at = Column(DateTime, default=lambda: datetime.now(timezone.utc))


class DeviceReport(Base):
    __tablename__ = "device_reports"
    id = Column(Integer, primary_key=True, autoincrement=True)
    device_id = Column(String(64), nullable=False, index=True)
    location = Column(String(128), nullable=False, default="未知地点", index=True)
    camera_id = Column(String(64), nullable=False, index=True)
    radar_dist = Column(Float, nullable=False)
    radar_x_mm = Column(Float, nullable=False, default=0.0)
    radar_y_mm = Column(Float, nullable=False, default=0.0)
    gimbal_pan = Column(Float, nullable=False)
    gimbal_tilt = Column(Float, nullable=False)
    gimbal_pan_raw = Column(Float, nullable=False, default=0.0)
    gimbal_tilt_raw = Column(Float, nullable=False, default=0.0)
    target_x = Column(Float, nullable=False)
    target_y = Column(Float, nullable=False)
    status = Column(String(16), nullable=False)

    # Algorithm Ext columns
    feedforward_pan = Column(Float, nullable=False, default=0.0)
    filtered_error = Column(Float, nullable=False, default=0.0)
    radar_vx = Column(Float, nullable=False, default=0.0)

    # Tracking mode/state columns
    tracking_mode = Column(String(32), nullable=False, default="STATE_SEARCHING")
    hover_active = Column(Integer, nullable=False, default=0)

    # Hardware health diagnostics columns
    uart3_drop = Column(Integer, nullable=False, default=0)
    uart3_sent = Column(Integer, nullable=False, default=0)
    uart3_q = Column(Integer, nullable=False, default=0)
    uart3_q_peak = Column(Integer, nullable=False, default=0)
    uart3_reject = Column(Integer, nullable=False, default=0)
    uart3_skip = Column(Integer, nullable=False, default=0)
    servo_drop = Column(Integer, nullable=False, default=0)
    uart3_ovf_reset = Column(Integer, nullable=False, default=0)
    
    reported_at = Column(DateTime, default=lambda: datetime.now(timezone.utc), index=True)


engine = create_async_engine(DATABASE_URL, echo=False)
async_session_factory = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

DEVICE_REPORT_COMPAT_COLUMNS = {
    "tracking_mode": "VARCHAR(32) NOT NULL DEFAULT 'STATE_SEARCHING'",
    "hover_active": "INTEGER NOT NULL DEFAULT 0",
    "feedforward_pan": "FLOAT NOT NULL DEFAULT 0.0",
    "filtered_error": "FLOAT NOT NULL DEFAULT 0.0",
    "radar_vx": "FLOAT NOT NULL DEFAULT 0.0",
    "uart3_drop": "INTEGER NOT NULL DEFAULT 0",
    "uart3_sent": "INTEGER NOT NULL DEFAULT 0",
    "uart3_q": "INTEGER NOT NULL DEFAULT 0",
    "uart3_q_peak": "INTEGER NOT NULL DEFAULT 0",
    "uart3_reject": "INTEGER NOT NULL DEFAULT 0",
    "uart3_skip": "INTEGER NOT NULL DEFAULT 0",
    "servo_drop": "INTEGER NOT NULL DEFAULT 0",
    "uart3_ovf_reset": "INTEGER NOT NULL DEFAULT 0",
    "radar_x_mm": "FLOAT NOT NULL DEFAULT 0.0",
    "radar_y_mm": "FLOAT NOT NULL DEFAULT 0.0",
    "gimbal_pan_raw": "FLOAT NOT NULL DEFAULT 0.0",
    "gimbal_tilt_raw": "FLOAT NOT NULL DEFAULT 0.0",
}


async def ensure_device_report_schema(conn) -> None:
    result = await conn.exec_driver_sql("PRAGMA table_info(device_reports)")
    existing_columns = {row[1] for row in result.fetchall()}

    for column_name, ddl in DEVICE_REPORT_COMPAT_COLUMNS.items():
        if column_name not in existing_columns:
            await conn.exec_driver_sql(
                f"ALTER TABLE device_reports ADD COLUMN {column_name} {ddl}"
            )


async def get_db() -> AsyncSession:  # type: ignore[misc]
    async with async_session_factory() as session:
        yield session


def serialize_device_report(
    report: DeviceReport,
    *,
    location_override: Optional[str] = None,
    iso_timestamp: bool = False,
) -> dict:
    return {
        "id": report.id,
        "device_id": report.device_id,
        "location": location_override or report.location,
        "camera_id": report.camera_id,
        "radar_dist": report.radar_dist,
        "radar_x_mm": report.radar_x_mm,
        "radar_y_mm": report.radar_y_mm,
        "gimbal_pan": report.gimbal_pan,
        "gimbal_tilt": report.gimbal_tilt,
        "gimbal_pan_raw": report.gimbal_pan_raw,
        "gimbal_tilt_raw": report.gimbal_tilt_raw,
        "target_x": report.target_x,
        "target_y": report.target_y,
        "status": report.status,
        "tracking_mode": report.tracking_mode,
        "hover_active": report.hover_active,
        "feedforward_pan": report.feedforward_pan,
        "filtered_error": report.filtered_error,
        "radar_vx": report.radar_vx,
        "uart3_drop": report.uart3_drop,
        "uart3_sent": report.uart3_sent,
        "uart3_q": report.uart3_q,
        "uart3_q_peak": report.uart3_q_peak,
        "uart3_reject": report.uart3_reject,
        "uart3_skip": report.uart3_skip,
        "servo_drop": report.servo_drop,
        "uart3_ovf_reset": report.uart3_ovf_reset,
        "reported_at": report.reported_at.isoformat() if iso_timestamp and report.reported_at else report.reported_at,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# WebSocket 连接管理器 (按摄像头权限过滤)
# ═══════════════════════════════════════════════════════════════════════════════

class ConnectionManager:
    def __init__(self) -> None:
        self._clients: list[dict] = []

    async def connect(self, ws: WebSocket, allowed_cameras: list[str]) -> None:
        await ws.accept()
        self._clients.append({
            "ws": ws,
            "allowed_cameras": allowed_cameras,
            "current_location": None,
        })

    def disconnect(self, ws: WebSocket) -> None:
        self._clients = [c for c in self._clients if c["ws"] is not ws]

    def set_current_location(self, ws: WebSocket, location: str) -> None:
        for c in self._clients:
            if c["ws"] is ws:
                c["current_location"] = location
                break

    async def broadcast(self, message: dict, location: str, camera_id: str) -> None:
        stale: list[WebSocket] = []
        for c in self._clients:
            allowed = c["allowed_cameras"]
            current = c["current_location"]
            has_perm = "all" in allowed or camera_id in allowed
            is_viewing = current == location
            if has_perm and is_viewing:
                try:
                    await c["ws"].send_json(message)
                except Exception:
                    stale.append(c["ws"])
        for ws in stale:
            self.disconnect(ws)


manager = ConnectionManager()

# ═══════════════════════════════════════════════════════════════════════════════
# JWT 工具函数
# ═══════════════════════════════════════════════════════════════════════════════

def create_access_token(username: str, role: str, allowed_cameras: list[str]) -> str:
    payload = {
        "sub": username,
        "role": role,
        "cameras": allowed_cameras,
        "exp": datetime.now(timezone.utc) + timedelta(hours=JWT_EXPIRE_HOURS),
    }
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)


def decode_access_token(token: str) -> dict:
    try:
        return jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token 已过期")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="无效 Token")


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security_scheme),
) -> dict:
    return decode_access_token(credentials.credentials)


async def require_admin(current_user: dict = Depends(get_current_user)) -> dict:
    if current_user.get("role") != "admin":
        raise HTTPException(status_code=403, detail="需要管理员权限")
    return current_user


# ═══════════════════════════════════════════════════════════════════════════════
# 数据老化后台任务
# ═══════════════════════════════════════════════════════════════════════════════

async def data_aging_task() -> None:
    while True:
        try:
            async with async_session_factory() as session:
                cutoff = datetime.now(timezone.utc) - timedelta(days=DATA_AGING_DAYS)
                await session.execute(
                    delete(DeviceReport).where(DeviceReport.reported_at < cutoff)
                )
                count_result = await session.execute(
                    select(func.count()).select_from(DeviceReport)
                )
                total = count_result.scalar() or 0
                if total > MAX_RECORDS:
                    overflow = total - MAX_RECORDS
                    oldest_ids = await session.execute(
                        select(DeviceReport.id).order_by(DeviceReport.id.asc()).limit(overflow)
                    )
                    ids_to_delete = [row[0] for row in oldest_ids.fetchall()]
                    if ids_to_delete:
                        await session.execute(
                            delete(DeviceReport).where(DeviceReport.id.in_(ids_to_delete))
                        )
                await session.commit()
        except Exception as exc:
            print(f"[data_aging] 清理异常: {exc}")
        await asyncio.sleep(AGING_INTERVAL_SECONDS)


# ═══════════════════════════════════════════════════════════════════════════════
# Pydantic 请求 / 响应模型
# ═══════════════════════════════════════════════════════════════════════════════

class LoginRequest(BaseModel):
    username: str
    password: str

class LoginResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    role: str
    allowed_cameras: list[str]

class CreateUserRequest(BaseModel):
    username: str = Field(..., min_length=2, max_length=64)
    password: str = Field(..., min_length=6, max_length=128)
    role: str = Field(default="user", pattern=r"^(admin|user)$")
    allowed_cameras: list[str] = Field(default_factory=list)

class UpdateUserCamerasRequest(BaseModel):
    username: str
    allowed_cameras: list[str]

class DeviceUploadRequest(BaseModel):
    device_id: str
    location: str = Field(default="未知地点", max_length=128)
    camera_id: str = Field(default="CAM_DEFAULT", max_length=64)
    radar_dist: float = Field(..., ge=0.0, le=6000.0)
    radar_x_mm: float = Field(default=0.0, ge=-6000.0, le=6000.0)
    radar_y_mm: float = Field(default=0.0, ge=0.0, le=6000.0)
    gimbal_pan: float = Field(..., ge=0.0, le=270.0)
    gimbal_tilt: float = Field(..., ge=0.0, le=180.0)
    gimbal_pan_raw: float = Field(default=0.0, ge=0.0, le=4095.0)
    gimbal_tilt_raw: float = Field(default=0.0, ge=0.0, le=2457.0)
    target_x: float = Field(..., ge=0.0, le=239.0)
    target_y: float = Field(..., ge=0.0, le=239.0)
    status: str = Field(..., pattern=r"^(VISION_STATUS_LOCKED|VISION_STATUS_PREDICTED|VISION_STATUS_LOST)$")
    
    # Hardware health diagnostics (optional for backward compatibility)
    uart3_drop: int = Field(default=0, ge=0, description="UART3 cumulative dropped bytes due to backpressure")
    uart3_sent: int = Field(default=0, ge=0, description="UART3 total transmitted bytes")
    uart3_q: int = Field(default=0, ge=0, le=768, description="Current UART3 ring buffer occupancy")
    uart3_q_peak: int = Field(default=0, ge=0, le=768, description="Peak UART3 ring buffer occupancy")
    uart3_reject: int = Field(default=0, ge=0, description="Count of rejected uploads due to queue saturation")
    uart3_skip: int = Field(default=0, ge=0, description="Skipped telemetry cycles due to high pressure")
    servo_drop: int = Field(default=0, ge=0, description="Dropped servo frames")
    uart3_ovf_reset: int = Field(default=0, ge=0, description="Overflow recovery resets (consecutive overflow events)")

    # Tracking state
    tracking_mode: str = Field(default="STATE_SEARCHING", max_length=32, description="FSM tracking state name")
    hover_active: int = Field(default=0, ge=0, le=1, description="Hover lock indicator")

    # Algorithm Ext: Feedforward & filtering metrics
    feedforward_pan: float = Field(default=0.0, description="Angular velocity feedforward compensation")
    filtered_error: float = Field(default=0.0, description="IIR filtered vision error")
    radar_vx: float = Field(default=0.0, description="Radar radial velocity on X axis")

class DeviceReportOut(BaseModel):
    id: int
    device_id: str
    location: str
    camera_id: str
    radar_dist: float
    radar_x_mm: float = 0.0
    radar_y_mm: float = 0.0
    gimbal_pan: float
    gimbal_tilt: float
    gimbal_pan_raw: float = 0.0
    gimbal_tilt_raw: float = 0.0
    target_x: float
    target_y: float
    status: str
    
    # Tracking state
    tracking_mode: str = "STATE_SEARCHING"
    hover_active: int = 0

    # Algorithm Ext Out
    feedforward_pan: float = 0.0
    filtered_error: float = 0.0
    radar_vx: float = 0.0

    # Hardware health diagnostics
    uart3_drop: int = 0
    uart3_sent: int = 0
    uart3_q: int = 0
    uart3_q_peak: int = 0
    uart3_reject: int = 0
    uart3_skip: int = 0
    servo_drop: int = 0
    uart3_ovf_reset: int = 0
    
    reported_at: datetime

class UserOut(BaseModel):
    id: int
    username: str
    role: str
    allowed_cameras: list[str]

class LocationIn(BaseModel):
    name: str = Field(..., min_length=1, max_length=128)
    description: str = Field(default="", max_length=512)

class LocationOut(BaseModel):
    id: int
    name: str
    description: str

class CameraIn(BaseModel):
    camera_name: str = Field(..., min_length=1, max_length=64)
    location_id: int

class CameraOut(BaseModel):
    id: int
    camera_name: str
    location_id: int
    location_name: str
    status: str


# ═══════════════════════════════════════════════════════════════════════════════
# 初始化 & 生命周期
# ═══════════════════════════════════════════════════════════════════════════════

async def init_db() -> None:
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
        await ensure_device_report_schema(conn)

    async with async_session_factory() as session:
        result = await session.execute(
            select(User).where(User.username == DEFAULT_ADMIN_USERNAME)
        )
        if result.scalar_one_or_none() is None:
            session.add(User(
                username=DEFAULT_ADMIN_USERNAME,
                password_hash=pwd_context.hash(DEFAULT_ADMIN_PASSWORD),
                role="admin",
                allowed_cameras='["all"]',
            ))
            await session.commit()
            print(f"[init] 已创建默认管理员: {DEFAULT_ADMIN_USERNAME}")

        # Seed guest account
        guest_exists = await session.execute(select(User).where(User.username == "guest"))
        if guest_exists.scalar_one_or_none() is None:
            session.add(User(
                username="guest",
                password_hash=pwd_context.hash("guest123"),
                role="visitor",
                allowed_cameras='["CAM_NORTH_01"]',
            ))
            await session.commit()
            print("[init] 已创建访客账号: guest")

        # Seed region_admin account
        ra_exists = await session.execute(select(User).where(User.username == "region_admin"))
        if ra_exists.scalar_one_or_none() is None:
            session.add(User(
                username="region_admin",
                password_hash=pwd_context.hash("admin123"),
                role="admin",
                allowed_cameras='["CAM_NORTH_01", "CAM_SOUTH_02"]',
            ))
            await session.commit()
            print("[init] 已创建区域管理员账号: region_admin")

        # Seed default locations & cameras if empty
        loc_count = await session.execute(select(func.count()).select_from(Location))
        if (loc_count.scalar() or 0) == 0:
            loc1 = Location(name="学校操场", description="校园内无人机监控区域")
            loc2 = Location(name="市中心", description="城区核心监控区域")
            loc3 = Location(name="郊区野外", description="郊区野外无人机监控区域")
            session.add_all([loc1, loc2, loc3])
            await session.flush()

            session.add_all([
                Camera(camera_name="CAM_NORTH_01", location_id=loc1.id, status="active"),
                Camera(camera_name="CAM_SOUTH_02", location_id=loc1.id, status="active"),
                Camera(camera_name="CAM_CENTER_03", location_id=loc1.id, status="active"),
                Camera(camera_name="CAM_CITY_01", location_id=loc2.id, status="active"),
                Camera(camera_name="CAM_WILD_01", location_id=loc3.id, status="active"),
            ])
            await session.commit()
            print("[init] 已创建默认地点和摄像头")


@asynccontextmanager
async def lifespan(app: FastAPI):
    await init_db()
    aging = asyncio.create_task(data_aging_task())
    print("[startup] 封神版 v3.0 — 强管控架构启动完毕")
    yield
    aging.cancel()
    await engine.dispose()


# ═══════════════════════════════════════════════════════════════════════════════
# FastAPI 应用实例
# ═══════════════════════════════════════════════════════════════════════════════

app = FastAPI(
    title="Drone Tracking WebCloud",
    version="3.0.0",
    description="低空无人机追踪系统 — 强管控 Location/Camera RBAC",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ═══════════════════════════════════════════════════════════════════════════════
# 路由：认证
# ═══════════════════════════════════════════════════════════════════════════════

@app.post("/api/auth/login", response_model=LoginResponse, tags=["Auth"])
async def login(body: LoginRequest, db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(User).where(User.username == body.username))
    user = result.scalar_one_or_none()
    if user is None or not pwd_context.verify(body.password, user.password_hash):
        raise HTTPException(status_code=401, detail="用户名或密码错误")

    cameras = json.loads(user.allowed_cameras) if user.allowed_cameras else []
    if user.role == "admin":
        cameras = ["all"]
    token = create_access_token(user.username, user.role, cameras)
    return LoginResponse(access_token=token, role=user.role, allowed_cameras=cameras)


# ═══════════════════════════════════════════════════════════════════════════════
# 路由：Admin — 地点管理 CRUD
# ═══════════════════════════════════════════════════════════════════════════════

@app.get("/api/admin/locations", tags=["Admin"])
async def admin_list_locations(db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    result = await db.execute(select(Location).order_by(Location.id.asc()))
    locs = result.scalars().all()
    return [LocationOut(id=l.id, name=l.name, description=l.description) for l in locs]


@app.post("/api/admin/locations", tags=["Admin"], status_code=201)
async def create_location(body: LocationIn, db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    exists = await db.execute(select(Location).where(Location.name == body.name))
    if exists.scalar_one_or_none():
        raise HTTPException(status_code=409, detail="地点名称已存在")
    loc = Location(name=body.name, description=body.description)
    db.add(loc)
    await db.commit()
    await db.refresh(loc)
    return {"message": f"地点 '{body.name}' 创建成功", "id": loc.id}


@app.delete("/api/admin/locations/{location_id}", tags=["Admin"])
async def delete_location(location_id: int, db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    result = await db.execute(select(Location).where(Location.id == location_id))
    loc = result.scalar_one_or_none()
    if not loc:
        raise HTTPException(status_code=404, detail="地点不存在")
    await db.delete(loc)
    await db.commit()
    return {"message": f"地点 '{loc.name}' 及其下属摄像头已删除"}


# ═══════════════════════════════════════════════════════════════════════════════
# 路由：Admin — 摄像头管理 CRUD
# ═══════════════════════════════════════════════════════════════════════════════

@app.get("/api/admin/cameras", tags=["Admin"])
async def admin_list_cameras(db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    result = await db.execute(select(Camera).order_by(Camera.id.asc()))
    cams = result.scalars().all()
    out = []
    for c in cams:
        await db.refresh(c, ["location"])
        out.append(CameraOut(
            id=c.id, camera_name=c.camera_name,
            location_id=c.location_id,
            location_name=c.location.name if c.location else "?",
            status=c.status,
        ))
    return out


@app.post("/api/admin/cameras", tags=["Admin"], status_code=201)
async def create_camera(body: CameraIn, db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    loc = await db.execute(select(Location).where(Location.id == body.location_id))
    if not loc.scalar_one_or_none():
        raise HTTPException(status_code=404, detail="所属地点不存在")
    exists = await db.execute(select(Camera).where(Camera.camera_name == body.camera_name))
    if exists.scalar_one_or_none():
        raise HTTPException(status_code=409, detail="摄像头名称已存在")
    cam = Camera(camera_name=body.camera_name, location_id=body.location_id)
    db.add(cam)
    await db.commit()
    await db.refresh(cam)
    return {"message": f"摄像头 '{body.camera_name}' 创建成功", "id": cam.id}


@app.delete("/api/admin/cameras/{camera_id}", tags=["Admin"])
async def delete_camera(camera_id: int, db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    result = await db.execute(select(Camera).where(Camera.id == camera_id))
    cam = result.scalar_one_or_none()
    if not cam:
        raise HTTPException(status_code=404, detail="摄像头不存在")
    await db.delete(cam)
    await db.commit()
    return {"message": f"摄像头 '{cam.camera_name}' 已删除"}


# ═══════════════════════════════════════════════════════════════════════════════
# 路由：Admin — 用户管理 CRUD
# ═══════════════════════════════════════════════════════════════════════════════

@app.post("/api/admin/users", tags=["Admin"], status_code=201)
async def create_user(body: CreateUserRequest, db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    exists = await db.execute(select(User).where(User.username == body.username))
    if exists.scalar_one_or_none():
        raise HTTPException(status_code=409, detail="该用户名已使用")
    cams_json = '["all"]' if body.role == "admin" else json.dumps(body.allowed_cameras, ensure_ascii=False)
    db.add(User(
        username=body.username,
        password_hash=pwd_context.hash(body.password),
        role=body.role,
        allowed_cameras=cams_json,
    ))
    await db.commit()
    return {"message": f"用户 '{body.username}' 创建成功", "role": body.role}


@app.get("/api/admin/users", response_model=list[UserOut], tags=["Admin"])
async def list_users(db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    result = await db.execute(select(User).where(User.username != 'guest').order_by(User.id.asc()))
    users = result.scalars().all()
    return [
        UserOut(id=u.id, username=u.username, role=u.role,
                allowed_cameras=json.loads(u.allowed_cameras) if u.allowed_cameras else [])
        for u in users
    ]


@app.put("/api/admin/users/cameras", tags=["Admin"])
async def update_user_cameras(body: UpdateUserCamerasRequest, db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    result = await db.execute(select(User).where(User.username == body.username))
    user = result.scalar_one_or_none()
    if not user:
        raise HTTPException(status_code=404, detail="用户不存在")
    if user.role == "admin":
        raise HTTPException(status_code=400, detail="管理员默认拥有全部权限")
    user.allowed_cameras = json.dumps(body.allowed_cameras, ensure_ascii=False)
    await db.commit()
    return {"message": f"用户 '{body.username}' 摄像头权限已更新"}


@app.delete("/api/admin/users/{username}", tags=["Admin"])
async def delete_user(username: str, db: AsyncSession = Depends(get_db), _admin: dict = Depends(require_admin)):
    if username == DEFAULT_ADMIN_USERNAME:
        raise HTTPException(status_code=400, detail="不能删除默认管理员")
    if username == 'guest':
        raise HTTPException(status_code=400, detail="不能删除系统内置访客组件")
    result = await db.execute(select(User).where(User.username == username))
    if not result.scalar_one_or_none():
        raise HTTPException(status_code=404, detail="用户不存在")
    await db.execute(delete(User).where(User.username == username))
    await db.commit()
    return {"message": f"用户 '{username}' 已删除"}


# ═══════════════════════════════════════════════════════════════════════════════
# 路由：公共 — 资产树 (地点 + 摄像头)
# ═══════════════════════════════════════════════════════════════════════════════

@app.get("/api/asset-tree", tags=["Asset"])
async def get_asset_tree(db: AsyncSession = Depends(get_db)):
    locs = (await db.execute(select(Location).order_by(Location.id))).scalars().all()
    tree = []
    for loc in locs:
        cams = (await db.execute(
            select(Camera).where(Camera.location_id == loc.id).order_by(Camera.id)
        )).scalars().all()
        tree.append({
            "id": loc.id,
            "name": loc.name,
            "description": loc.description,
            "cameras": [{"id": c.id, "camera_name": c.camera_name, "status": c.status} for c in cams],
        })
    return tree


@app.get("/api/locations", tags=["Asset"])
async def get_locations(db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(Location.name).order_by(Location.id))
    return {"locations": [row[0] for row in result.fetchall()]}


# ═══════════════════════════════════════════════════════════════════════════════
# 路由：硬件数据上报 (强管控校验)
# ═══════════════════════════════════════════════════════════════════════════════

@app.post("/api/device/upload", tags=["Device"], status_code=201)
async def device_upload(body: DeviceUploadRequest, db: AsyncSession = Depends(get_db)):
    cam_result = await db.execute(select(Camera).where(Camera.camera_name == body.camera_id))
    cam = cam_result.scalar_one_or_none()
    if cam is None:
        raise HTTPException(status_code=403, detail=f"未注册的摄像头 '{body.camera_id}'，数据已拦截")

    await db.refresh(cam, ["location"])
    real_location = cam.location.name if cam.location else body.location
    resolved_radar_dist = body.radar_dist
    if (resolved_radar_dist <= 0.0) and ((body.radar_x_mm != 0.0) or (body.radar_y_mm != 0.0)):
        resolved_radar_dist = math.hypot(body.radar_x_mm, body.radar_y_mm)

    report = DeviceReport(
        device_id=body.device_id,
        location=real_location,
        camera_id=body.camera_id,
        radar_dist=resolved_radar_dist,
        radar_x_mm=body.radar_x_mm,
        radar_y_mm=body.radar_y_mm,
        gimbal_pan=body.gimbal_pan,
        gimbal_tilt=body.gimbal_tilt,
        gimbal_pan_raw=body.gimbal_pan_raw,
        gimbal_tilt_raw=body.gimbal_tilt_raw,
        target_x=body.target_x,
        target_y=body.target_y,
        status=body.status,

        # Tracking state
        tracking_mode=body.tracking_mode,
        hover_active=body.hover_active,

        # Algorithm Ext
        feedforward_pan=body.feedforward_pan,
        filtered_error=body.filtered_error,
        radar_vx=body.radar_vx,

        # Hardware health diagnostics
        uart3_drop=body.uart3_drop,
        uart3_sent=body.uart3_sent,
        uart3_q=body.uart3_q,
        uart3_q_peak=body.uart3_q_peak,
        uart3_reject=body.uart3_reject,
        uart3_skip=body.uart3_skip,
        servo_drop=body.servo_drop,
        uart3_ovf_reset=body.uart3_ovf_reset,
    )
    db.add(report)
    await db.commit()
    await db.refresh(report)

    broadcast_payload = {
        "type": "realtime",
        "data": serialize_device_report(report, location_override=real_location, iso_timestamp=True),
    }
    await manager.broadcast(broadcast_payload, real_location, report.camera_id)

    return {"message": "ok", "id": report.id}


# ═══════════════════════════════════════════════════════════════════════════════
# 路由：历史数据查询
# ═══════════════════════════════════════════════════════════════════════════════

@app.get("/api/device/history", response_model=list[DeviceReportOut], tags=["Device"])
async def device_history(
    device_id: Optional[str] = Query(None),
    location: Optional[str] = Query(None),
    camera_id: Optional[str] = Query(None),
    limit: int = Query(50, ge=1, le=500),
    offset: int = Query(0, ge=0),
    db: AsyncSession = Depends(get_db),
    _user: dict = Depends(get_current_user),
):
    stmt = select(DeviceReport).order_by(DeviceReport.id.desc())
    if device_id:
        stmt = stmt.where(DeviceReport.device_id == device_id)
    if location:
        stmt = stmt.where(DeviceReport.location == location)
    if camera_id:
        stmt = stmt.where(DeviceReport.camera_id == camera_id)
    stmt = stmt.offset(offset).limit(limit)

    result = await db.execute(stmt)
    rows = result.scalars().all()
    return [DeviceReportOut(**serialize_device_report(r)) for r in rows]


# ═══════════════════════════════════════════════════════════════════════════════
# 路由：WebSocket 实时推送
# ═══════════════════════════════════════════════════════════════════════════════

@app.websocket("/ws/dashboard")
async def ws_dashboard(ws: WebSocket, token: str = Query(...)):
    try:
        payload = decode_access_token(token)
    except HTTPException:
        await ws.close(code=4001, reason="认证失败")
        return

    allowed_cameras = payload.get("cameras", [])
    await manager.connect(ws, allowed_cameras)
    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
                if msg.get("action") == "switch_location":
                    loc = msg.get("location", "")
                    if loc:
                        manager.set_current_location(ws, loc)
            except (json.JSONDecodeError, AttributeError):
                pass
    except WebSocketDisconnect:
        manager.disconnect(ws)


# ═══════════════════════════════════════════════════════════════════════════════
# 静态 HTML 托管
# ═══════════════════════════════════════════════════════════════════════════════

@app.get("/", include_in_schema=False)
async def root():
    return RedirectResponse(url="/login.html")

@app.get("/login.html", include_in_schema=False)
async def page_login():
    return FileResponse(os.path.join(BASE_DIR, "login.html"), media_type="text/html")

@app.get("/dashboard.html", include_in_schema=False)
async def page_dashboard():
    return FileResponse(os.path.join(BASE_DIR, "dashboard.html"), media_type="text/html")

@app.get("/admin.html", include_in_schema=False)
async def page_admin():
    return FileResponse(os.path.join(BASE_DIR, "admin.html"), media_type="text/html")


# 健康检查
# ═══════════════════════════════════════════════════════════════════════════════

@app.get("/health", tags=["System"])
async def health():
    return {"status": "ok", "version": "3.0.0", "ts": datetime.now(timezone.utc).isoformat()}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
