import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from app.bridges import get_bridge
from app.config import settings
from app.routers import api, ws
from app.services.telemetry_hub import telemetry_hub

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("amr_web.main")


@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("Khởi động AMR Omni Web Backend...")
    bridge = get_bridge()
    await bridge.start()
    await telemetry_hub.start()
    yield
    logger.info("Dừng AMR Omni Web Backend...")
    await telemetry_hub.stop()
    await bridge.stop()


app = FastAPI(
    title="AMR Omni Web Interface API",
    description="Hệ thống backend điều khiển, giám sát luồng dữ liệu Jetson-STM32 và bản đồ LiDAR",
    version="1.0.0",
    lifespan=lifespan,
)

# Cấu hình CORS để Next.js frontend truy cập thông suốt
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.middleware("http")
async def add_no_cache_headers(request, call_next):
    response = await call_next(request)
    # Ngăn trình duyệt cache file index.html hoặc các route SPA để luôn tải bản mới nhất sau khi build
    if request.url.path in ("/", "/index.html", "/index.txt") or request.url.path.endswith(".html"):
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate, max-age=0"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
    return response

# Đăng ký các router
app.include_router(api.router)
app.include_router(ws.router)


@app.get("/health")
async def health_check():
    return {"status": "ok", "service": "amr_omni_backend"}


# Phục vụ Frontend tĩnh nếu đã được build vào thư mục static (phải đặt sau API routes)
static_path = settings.STATIC_DIR
if static_path.exists() and (static_path / "index.html").exists():
    logger.info(f"Phục vụ frontend tĩnh từ: {static_path}")
    app.mount("/", StaticFiles(directory=str(static_path), html=True), name="static")
else:
    logger.info("Chưa tìm thấy frontend build trong static/, hoạt động ở chế độ API/WebSocket standalone.")

