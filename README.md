# Ponderada Turtlebot Teleoperado

## Descrição da Atividade
Fazer o setup e interagir com o turtlebot, compreendendo os conceitos básico para uso do ROS em rede e dos pacotes para interação com o robô.

## Vídeo da Atividade
Para visualizar na prática, assista esse [vídeo]()

## Instruções para Execução

### Pré-requisitos
- Ubuntu 20.04
- Python 3 ou superior
- Ros 2
- Webots

### Clone o Repositório
``` git clone https://github.com/cecigonca/ponderada-turtlebot-teleoperado.git```

### Ambiente Virtual
Na janela do terminal (Linux) digite os seguintes comandos para cada dependência
```python3 -m venv venv```

```source ./venv/bin/activate```

### Importando as Dependências
```python3 -m pip install -r requirements.txt```

### Executando
Em um terminal rode o seguinte comando

```ros2 launch webots_ros2_turtlebot robot_launch.py```

Enquando isso, em outro terminal navegue até o caminho e rode os comandos seguintes

```/../ponderada-turtlebot-teleoperado/```

```colcon build```

```source install/local_setup.bash```

```python3 mover.py```

## Funcionamento da Atividade

↑ : Mover para frente

↓ : Mover para trás

← : Mover para esquerda

→ : Mover para direita

Tecla 'Espaço' : zera as velocidades

Tecla 'Enter' : encerra o programa

## Funções 
**__init__**: responsável por iniciar o ROS, configuras as velocidades iniciais e armazenar as configurações do terminal.

**update**: verifica se alguma tecla está sendo pressionada, se estiver, processa usando a função 'processKey'

**getKey**: lê a tecla do terminal e retorna essa tecla para ser processada.

**processKey**: ajusta as velocidades com base na tecla pressionada e chama a função 'publishTwist'.

**publishTwist**: atualiza a velocidade do Turtlebot.

**main**: inicia o nó (ROS) rodando até que seja encerrado pelo comando ou interrupção.



