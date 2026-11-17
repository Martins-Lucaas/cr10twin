#!/usr/bin/env bash
# Instalação nativa do workspace no Ubuntu 22.04; execute como usuário normal.
set -eo pipefail

if [ "$(id -u)" -eq 0 ]; then
    echo 'Execute como usuário normal, sem sudo; o script solicitará sudo quando necessário.' >&2
    exit 1
fi
if [ ! -r /etc/os-release ]; then
    echo 'Não foi possível identificar o sistema operacional.' >&2
    exit 1
fi
. /etc/os-release
if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 22.04 ]; then
    echo 'Este instalador requer Ubuntu 22.04.' >&2
    exit 1
fi

echo 'Atualizando índices e instalando ferramentas básicas...'
sudo apt update
sudo apt install -y software-properties-common curl ca-certificates git python3-pip
sudo add-apt-repository -y universe

# Identidade e autenticação não são necessárias para clonar um repositório público.
if [ -t 0 ]; then
    read -r -p 'Configurar nome e e-mail globais do Git para commits? [s/N] ' configure_git
    if [[ "$configure_git" =~ ^[sS]([iI][mM])?$ ]]; then
        read -r -p 'Nome para os commits: ' git_name
        read -r -p 'E-mail para os commits: ' git_email
        if [ -n "$git_name" ] && [ -n "$git_email" ]; then
            git config --global user.name "$git_name"
            git config --global user.email "$git_email"
        else
            echo 'Nome ou e-mail vazio: identidade Git não alterada.'
        fi
    fi
    read -r -p 'Autenticar no GitHub agora com o GitHub CLI? [s/N] ' configure_github
    if [[ "$configure_github" =~ ^[sS]([iI][mM])?$ ]]; then
        sudo apt install -y gh
        if ! gh auth login --hostname github.com --git-protocol https --web; then
            echo 'Autenticação não concluída. O clone público continuará sem login.' >&2
        fi
    fi
else
    echo 'Entrada não interativa: identidade Git e autenticação GitHub não serão alteradas.'
fi

workspace="$HOME/cr10twin"
if [ -e "$workspace" ] || [ -L "$workspace" ]; then
    if [ ! -d "$workspace" ] || ! git -C "$workspace" rev-parse --show-toplevel >/dev/null 2>&1 ||
       [ "$(git -C "$workspace" rev-parse --show-toplevel)" != "$(cd "$workspace" && pwd -P)" ]; then
        echo "O destino $workspace já existe e não é a raiz de um clone Git; nada foi sobrescrito." >&2
        exit 1
    fi
    origin=$(git -C "$workspace" remote get-url origin 2>/dev/null || true)
    case "$origin" in
        https://github.com/Martins-Lucaas/cr10twin|https://github.com/Martins-Lucaas/cr10twin.git|git@github.com:Martins-Lucaas/cr10twin|git@github.com:Martins-Lucaas/cr10twin.git|ssh://git@github.com/Martins-Lucaas/cr10twin|ssh://git@github.com/Martins-Lucaas/cr10twin.git) ;;
        *) echo "O destino já existe, mas seu origin não é o repositório esperado: $origin" >&2; exit 1 ;;
    esac
    echo "Usando o clone existente em $workspace, sem atualizar seus arquivos."
else
    git clone --recursive https://github.com/Martins-Lucaas/cr10twin.git "$workspace"
fi
cd "$workspace"

# Um clone usa origin para pull; contribuições devem ir para um fork.
# Um pushurl personalizado preexistente não é substituído automaticamente.
if git config --get-all remote.origin.pushurl >/dev/null; then
    echo 'O origin já tem pushurl personalizado; confira se ele não aponta para o repositório principal.'
else
    git remote set-url --push origin no_push://disabled
    echo 'Push pelo origin bloqueado; envie branches ao fork e abra pull requests.'
fi

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -qx 'install ok installed'; then
    echo 'Configurando o repositório oficial do ROS 2...'
    ros_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
    ros_deb=$(mktemp --suffix=.deb)
    trap 'rm -f "$ros_deb"' EXIT
    curl -fsSL -o "$ros_deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_version}/ros2-apt-source_${ros_version}.jammy_all.deb"
    sudo dpkg -i "$ros_deb"
fi

echo 'Instalando ROS 2 Humble e dependências do projeto...'
sudo apt update
sudo apt install -y \
    ros-humble-desktop ros-dev-tools python3-rosdep python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control ros-humble-xacro ros-humble-joint-state-publisher-gui \
    ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-control-msgs \
    ros-humble-admittance-controller ros-humble-kinematics-interface-kdl \
    ros-humble-force-torque-sensor-broadcaster python3-tk python3-pil python3-vtk9 \
    python3-numpy python3-opencv python3-serial python3-matplotlib python3-pytest

sudo usermod -aG dialout "$(id -un)"
if git submodule status --recursive | grep -Eq '^[+U]'; then
    echo 'Um submódulo existente diverge do commit esperado ou está em conflito; resolva-o antes de continuar.' >&2
    exit 1
fi
git submodule update --init --recursive
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
if ! python3 -c 'import numpy; assert int(numpy.__version__.split(".")[0]) < 2' 2>/dev/null; then
    echo 'NumPy 1.x não está disponível neste Python. Resolva o conflito antes de continuar (cv_bridge do Humble usa NumPy 1.x).' >&2
    exit 1
fi
covvi_version=$(python3 -c 'from importlib.metadata import version, PackageNotFoundError
try:
    print(version("covvi-eci"))
except PackageNotFoundError:
    pass')
if [ -z "$covvi_version" ]; then
    python3 -m pip install --user 'covvi-eci==1.1.6'
elif [ "$covvi_version" != '1.1.6' ]; then
    echo "covvi-eci $covvi_version já está instalado; o projeto exige 1.1.6. Resolva o conflito antes de continuar." >&2
    exit 1
fi
echo 'Compilando o workspace...'
colcon build --symlink-install

cat <<'EOF'

Instalação e compilação concluídas. Em cada novo terminal, execute:
  cd ~/cr10twin
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch touch_pack tactile_cell.launch.py

Câmera MediaPipe e WBAN/KINESIS para IMUs têm dependências adicionais;
consulte o tutorial antes de habilitá-las. Saia e entre novamente na sessão
para ativar o acesso às portas seriais pelo grupo dialout.
EOF
