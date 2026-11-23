# Transmite a webcam do Windows como MJPEG para o container (Docker Desktop
# não expõe /dev/video). O container lê TOUCH_CAMERA_URL (docker-compose.windows.yml).
# Uso:  .\tools\windows_webcam_stream.ps1  [-Camera "Nome da webcam"]
# Liste as câmeras com:  ffmpeg -list_devices true -f dshow -i dummy
param([string]$Camera = 'Full HD webcam', [int]$Port = 8090)

# -listen 1 atende um cliente e sai; o loop deixa o botão CAM ligar/desligar à vontade.
while ($true) {
    # rtbufsize pequeno: sem cliente o buffer enche; ao conectar, pouco quadro velho
    ffmpeg -hide_banner -loglevel error -rtbufsize 1M -f dshow -video_size 640x480 -framerate 30 `
        -i "video=$Camera" -f mpjpeg -q:v 5 -listen 1 "http://127.0.0.1:$Port/cam"
}
