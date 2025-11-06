# 🚇 Scanner de Trilhos de Elevador

Sistema automatizado para medição de alinhamento, vibração e integridade estrutural de trilhos de elevadores.

![Version](https://img.shields.io/badge/version-1.0-blue)
![Platform](https://img.shields.io/badge/platform-ESP32-green)
![License](https://img.shields.io/badge/license-MIT-orange)

---

## 📋 Sumário

- [Visão Geral](#-visão-geral)
- [Hardware](#-hardware)
- [Instalação](#-instalação)
- [Bibliotecas Necessárias](#-bibliotecas-necessárias)
- [Uso](#-uso)
- [Estrutura de Arquivos](#-estrutura-de-arquivos)
- [Análise de Dados](#-análise-de-dados)
- [Troubleshooting](#-troubleshooting)

---

## 🎯 Visão Geral

Este projeto foi desenvolvido para automatizar a inspeção de trilhos de elevadores, percorrendo verticalmente o trilho enquanto coleta dados sobre:

- **Alinhamento**: Distância do trilho através de 3 sensores ultrassônicos
- **Vibração**: Aceleração e rotação através de acelerômetro/giroscópio
- **Posição**: Rastreamento preciso da posição vertical

Os dados são salvos em cartão SD no formato CSV para análise posterior.

### Características

✅ Movimentação sincronizada de 4 motores de passo  
✅ Medição em tempo real com 3 sensores ultrassônicos  
✅ Monitoramento de vibração em 3 eixos  
✅ Armazenamento automático em SD Card  
✅ Interface serial para controle e monitoramento  
✅ Detecção automática de problemas críticos  

---

## 🔧 Hardware

### Componentes Principais

| Componente | Modelo | Quantidade | Função |
|------------|--------|------------|--------|
| Microcontrolador | ESP32 DevKit C V4 | 1 | Controle central |
| Motor de Passo | NEMA 17 (1.8°/passo) | 4 | Movimentação das rodinhas |
| Driver de Motor | A4988 | 4 | Controle dos motores |
| Sensor Ultrassônico | HC-SR04 | 3 | Medição de distância/alinhamento |
| Acelerômetro/Giroscópio | MPU6050 | 1 | Medição de vibração |
| Leitor SD | MicroSD Card Module | 1 | Armazenamento de dados |
| Bateria | LiPo/Li-ion | 1 | Alimentação portátil |

### Esquema de Conexões

#### Motores de Passo (A4988)

```
Motor 1: STEP=GPIO25, DIR=GPIO26
Motor 2: STEP=GPIO27, DIR=GPIO14
Motor 3: STEP=GPIO12, DIR=GPIO13
Motor 4: STEP=GPIO32, DIR=GPIO33

Alimentação: VMOT=12-24V, VDD=3.3V
```

#### Sensores Ultrassônicos (HC-SR04)

```
Sensor 1: TRIG=GPIO15, ECHO=GPIO2
Sensor 2: TRIG=GPIO4,  ECHO=GPIO16
Sensor 3: TRIG=GPIO17, ECHO=GPIO18

Alimentação: VCC=5V
```

#### Acelerômetro (MPU6050)

```
I2C: SDA=GPIO22, SCL=GPIO21
Alimentação: VCC=3.3V
Endereço: 0x68 (padrão)
```

#### Cartão SD

```
SPI: CS=GPIO5, MOSI=GPIO23, MISO=GPIO19, SCK=GPIO18
Alimentação: VCC=3.3V
```

### Diagrama Wokwi

Acesse o projeto completo no Wokwi: [Link do Projeto](https://wokwi.com/projects/440591027508086785)
