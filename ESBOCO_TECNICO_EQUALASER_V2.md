# ESBOÇO TÉCNICO AVANÇADO DO PROJETO: EQUALASER (v2.5)
**Sistema de Vigilância e Metrologia em Campo a Laser Offline para Conjuntos Motor Elétrico × Bomba Centrífuga**  
*Referência da Demanda: Plataforma Saga SENAI de Inovação — Demanda da Indústria #13821*  
*Instituição: SENAI CIMATEC (Bahia)*  

---

## 1. Resumo Executivo e Proposta de Valor
O **Equalaser v2.5** é um sistema preditivo e metrológico industrial de baixo custo, projetado para monitoramento contínuo carcaça-a-carcaça de conjuntos rotativos (motores elétricos trifásicos acoplados a bombas centrífugas).

### Proposta de Arquitetura Híbrida:
1. **Vigilância 24/7 Autônoma na Máquina:** Dispositivo permanentemente fixado às carcaças estacionárias (em conformidade com a NR-12), operando amostragem contínua a 250 Hz, histerese temporal anti-falso positivo, sinalização local Andon (LEDs + Buzzer) e gravação ininterrupta de eventos em cartão MicroSD com relógio RTC DS3231.
2. **Coleta de Rota Offline (Air-Gapped):** Durante as rondas periódicas de manutenção preditiva, o inspetor conecta qualquer smartphone ou tablet à rede Wi-Fi local fechada do ESP32 (`EQUALASER_GATEWAY`, IP `192.168.4.1`), visualizando o painel em tempo real, analisando a tendência histórica (Curva P-F), emitindo o laudo técnico em PDF e exportando o histórico `.csv` sem depender da rede corporativa de TI ou de acesso à nuvem.

---

## 2. Princípio Metrológico e Cinemática de Acoplamento Desacoplada

### 2.1. Desacoplamento entre Translação e Rotação
Ao monitorar os receptores laterais nas posições 9h (esquerda) e 3h (direita), o sistema separa matematicamente a translação paralela da inclinação angular:
* **Deslocamento Resultante em cada sensor:**
  $$\Delta d_{9h} = \sqrt{\Delta X_{9h}^2 + \Delta Y_{9h}^2}, \quad \Delta d_{3h} = \sqrt{\Delta X_{3h}^2 + \Delta Y_{3h}^2}$$
* **Translação Paralela Pura (Radial):**
  $$\Delta d_{\text{trans}} = \frac{\Delta d_{9h} + \Delta d_{3h}}{2}$$
* **Inclinação Angular ($\theta$):**
  $$\theta = \arctan\left(\frac{\Delta Y_{9h} - \Delta Y_{3h}}{L_{\text{apoios}}}\right)$$
* **Deslocamento Real no Plano do Acoplamento:**
  $$\Delta d_{\text{acoplamento}} = \Delta d_{\text{trans}} + \left|\tan(\theta) \cdot L_{\text{acoplamento}}\right|$$

### 2.2. Prescrição Mecânica de Calços (*Shims*)
Para orientar a intervenção mecânica corretiva da equipe de manutenção na parada do ativo:
* **Calço Pé Dianteiro ($S_1$):**
  $$S_1 = -\left(\frac{\Delta Y_{9h} + \Delta Y_{3h}}{2}\right) - \left(\tan(\theta) \cdot L_{\text{pe\_diant}}\right)$$
* **Calço Pé Traseiro ($S_2$):**
  $$S_2 = -\left(\frac{\Delta Y_{9h} + \Delta Y_{3h}}{2}\right) - \left(\tan(\theta) \cdot (L_{\text{pe\_diant}} + L_{\text{apoios}})\right)$$

---

## 3. Arquitetura Eletrônica, BOM e Requisitos Térmicos

| Item | Componente | Especificação Técnica | Função no Sistema |
| :--- | :--- | :--- | :--- |
| **01** | Microcontrolador | ESP32 DevKit v1 (Dual-Core 240 MHz, 520 KB SRAM) | Core 0: Amostragem a 250 Hz. Core 1: WebServer e SoftAP. |
| **02** | Diodos Laser | 650 nm, 5 mW, foco colimável | Feixes de referência laterais com modulação síncrona. |
| **03** | Sensores Ópticos | 8× Fototransistores TEPT4400 (arranjo quadrante) | Leitura bidimensional diferencial $(X, Y)$ nos lados 9h e 3h. |
| **04** | Filtro Óptico | Acrílico vermelho translúcido 650 nm + difusor | Rejeição de luz ambiente e linearização do spot laser. |
| **05** | Relógio RTC | DS3231 I2C com bateria moeda CR2032 | Timestamp cronológico offline ininterrupto. |
| **06** | Datalogger | Módulo leitor MicroSD SPI + Cartão Industrial 8GB | Caixa-preta local gravando `/EQUALASER.CSV`. |
| **07** | Driver Buzzer | Transistor NPN BC547 + Resistor $1\text{ k}\Omega$ + 1N4148 | Acionamento seguro do alarme acústico 5V sem sobrecarga no ESP32. |
| **08** | Ímãs de Fixação | Neodímio grau **N35SH ou N42H** ($T_{\max} \ge 120\text{--}150^\circ\text{C}$) | Fixação magnética de alta rigidez imune ao calor da carcaça. |
| **09** | Encapsulamento | Polímero **ASA ou Policarbonato (PC)** | Resistência a radiação UV, calor ($>90^\circ\text{C}$) e névoa química. |

---

## 4. Processamento de Sinal e Anti-Aliasing (250 Hz)
* **Teorema de Nyquist-Shannon:** Motores de 2 polos a 3.500 RPM vibram a $58{,}3\text{ Hz}$. A taxa de amostragem anterior de 10 Hz sofria grave *aliasing*. A versão v2.5 implementa timer de 4 ms (**250 Hz**) no Core 0 via FreeRTOS.
* **Componente AC (Índice de Vibração Óptica):** A componente alternada $V_{AC}(t) = V(t) - V_{DC}$ é calculada em RMS, fornecendo um indicador de severidade vibracional global e detecção precoce de soltura estrutural de base sem custo adicional de acelerômetros.
* **Leitura Diferencial Síncrona:** Laser ON e Laser OFF são alternados a 250 Hz, cancelando ruídos de $100/120\text{ Hz}$ de luminárias LED/fluorescentes industriais.
