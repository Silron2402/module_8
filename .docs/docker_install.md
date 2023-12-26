# Примечание

В данном примере используется аналогичный с предыдущим модулем контейнер.
В случае использования маломощного ПК рекомендуется нативная установка ОС ubuntu 20.04.

# Установка

```bash
curl -fsSL https://get.docker.com | sh
```

```bash
sudo usermod -aG docker $USER
```

```bash
sudo systemctl disable --now docker.service docker.socket
```

```bash
sudo apt-get install -y uidmap
```

```bash
dockerd-rootless-setuptool.sh install --force
```

# Сборка

Для сборки контейнера необходимо выполнить команду:

```bash
docker build -t px4_simulation:latest .
```

# Запуск

Перед запуском выполните команду `xhost+` для разрешения контейнеру доступу к экрану через **xserver**.

Для запуска предлагается воспользоваться скриптом из папки scripts

```bash
./start_sim_docker.sh -c <container_name>
```

Например:

```bash
./start_sim_docker.sh -c px4_simulation:latest
```