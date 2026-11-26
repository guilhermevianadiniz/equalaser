# EQUALASER 🎯⚡

> **Sistema de Vigilância e Metrologia Preditiva a Laser Offline para Conjuntos Motor Elétrico × Bomba Centrífuga**  
> *Plataforma Saga SENAI de Inovação — Demanda da Indústria #13821*  
> *Escola: SENAI CIMATEC (Salvador - BA)*

---

## 📌 Visão Geral do Projeto

O **Equalaser** é uma solução de manutenção preditiva e metrologia industrial de baixo custo desenvolvida para detectar, quantificar e diagnosticar precocemente desalinhamentos verticais e horizontais entre eixos de motores elétricos e bombas centrífugas em plena operação.

Diferentemente dos alinhadores estáticos convencionais — que exigem a parada total do ativo, bloqueio de energias perigosas (LOTO) e remoção das proteções mecânicas —, o Equalaser opera pelo método **carcaça-a-carcaça** nas partes estacionárias rígidas adjacentes ao acoplamento, garantindo **100% de conformidade com a norma de segurança NR-12**.

---

## 🏛️ Proposta de Arquitetura Híbrida

O sistema combina dois pilares operacionais para resolver a restrição de independência de rede corporativa de TI:

1. **Vigilância 24/7 Autônoma na Máquina:**
   - Instalação contínua nas carcaças por fixação magnética de neodímio térmico (**N35SH / N42H**, até 150 °C).
   - Processamento de alta velocidade a **250 Hz (4 ms)** com filtro *anti-aliasing* contra as harmônicas de rotação do motor (29,1 Hz e 58,3 Hz).
   - Histerese temporal (4,0 s) contra falsos alarmes causados por transitórios ou choques mecânicos.
   - Sinalização física local em padrão Andon (LEDs Verde, Amarelo e Vermelho + Buzzer industrial).
   - Caixa-preta local gravando histórico ininterrupto em **Cartão MicroSD** com timestamp de **RTC DS3231**.

2. **Coleta de Rota Offline (*Air-Gapped*):**
   - Durante as rondas periódicas de manutenção, o inspetor conecta qualquer smartphone, tablet ou notebook ao ponto de acesso Wi-Fi dedicado do microcontrolador (`EQUALASER_GATEWAY`, IP `192.168.4.1`).
   - A interface web responsiva roda direto da memória Flash do ESP32 (sem internet, nuvem ou necessidade de instalar aplicativos).
   - Emissão de **Laudo Técnico de Inspeção Preditiva** (pronto para PDF/impressão), visualização da **Curva P-F** e exportação do arquivo `.csv`.

---

## 🔬 Metrologia e Cinemática Desacoplada

O Equalaser utiliza dois pares ópticos simétricos instalados nas posições **9h** (lateral esquerda) e **3h** (lateral direita), separando matematicamente translação de rotação angular:

- **Deslocamento Radial na Carcaça:**  
  $$\Delta d = \sqrt{\Delta X^2 + \Delta Y^2}$$
- **Translação Paralela Pura:**  
  $$\Delta d_{\text{trans}} = \frac{\Delta d_{9h} + \Delta d_{3h}}{2}$$
- **Inclinação Angular ($\theta$):**  
  $$\theta = \arctan\left(\frac{\Delta Y_{9h} - \Delta Y_{3h}}{L_{\text{apoios}}}\right)$$
- **Deslocamento Projetado no Plano do Acoplamento:**  
  $$\Delta d_{\text{acoplamento}} = \Delta d_{\text{trans}} + \left|\tan(\theta) \cdot L_{\text{acoplamento}}\right|$$
- **Prescrição Mecânica de Calços (*Shims*):**  
  Calcula a espessura exata de calço a inserir ou remover nos pés dianteiros ($S_1$) e traseiros ($S_2$) na parada programada.
- **Índice Auxiliar de Vibração (RMS AC):**  
  Calcula a componente oscilatória de alta frequência do feixe laser para diagnóstico de soltura estrutural sem acelerômetro externo.

---

## 📁 Estrutura de Arquivos do Repositório

```
equalaser/
├── index.html                   # Interface Web completa (Dashboard, SVG dinâmico, Laudo e Simulador)
├── firmware_esp32.ino           # Firmware em C/C++ para ESP32 (FreeRTOS, WebServer, LittleFS e ADC)
├── ESBOCO_TECNICO_EQUALASER_V2.md # Memorial de cálculo, BOM industrial e arquitetura de engenharia
├── demanda.docx                 # Especificação da Demanda da Indústria #13821 (SENAI)
└── README.md                    # Documentação principal do projeto
```

---

## 🚀 Como Executar

### 1. Testando a Interface Web / Simulador (Sem Hardware)
1. Basta abrir o arquivo `index.html` em qualquer navegador web moderno (Google Chrome, Microsoft Edge, Firefox).
2. O simulador matemático interno entrará em ação automaticamente, permitindo testar:
   - Os 6 cenários de falha (Normal, Térmica, Angular, Pé Manco, Rampa de Degradação de 15s e Choque Transitório).
   - O diagrama cinemático vetorial (SVG) com deformação em tempo real do acoplamento flexível.
   - A calibração de Linha de Base (Tara) e a emissão de laudo técnico em PDF (`Ctrl + P` ou botão na tela).

### 2. Gravando o Firmware no ESP32
1. Abra a **Arduino IDE 2.x**.
2. Instale o pacote de placas ESP32 por meio do Gerenciador de Placas (*Espressif Systems*).
3. Selecione a placa **ESP32 Dev Module** (ou NodeMCU-32S).
4. Abra o arquivo `firmware_esp32.ino`.
5. Conecte o ESP32 via USB e faça o upload do sketch.
6. Grave o arquivo `index.html` na partição LittleFS do microcontrolador (via plugin *ESP32 LittleFS Data Upload* ou servindo da memória interna).
7. Conecte seu dispositivo na rede Wi-Fi `EQUALASER_GATEWAY` (senha: `senai1234`) e acesse `http://192.168.4.1`.

---

## 🛠️ Especificações de Componentes (BOM Industrial)

- **Microcontrolador:** ESP32 DevKit v1 Dual-Core 240 MHz (Core 0 para amostragem a 250 Hz; Core 1 para WebServer).
- **Emissores:** Diodos Laser Vermelho 650 nm, 5 mW, foco colimável com modulação síncrona diferencial (rejeição de luz ambiente).
- **Receptores:** Matriz de 8 fototransistores TEPT4400 em arranjo quadrante com janela acrílica passa-faixa 650 nm.
- **Datalogger:** Módulo Cartão MicroSD SPI + Relógio RTC de alta precisão DS3231 I2C com bateria moeda CR2032.
- **Sinalização:** Torre Andon de LEDs de alto brilho + Buzzer ativo 5 V chaveado por transistor NPN (BC547).
- **Carcaça Mecânica:** Polímero **ASA ou Policarbonato (PC)** com resistência térmica $> 95^\circ\text{C}$.
- **Fixação:** Ímãs de Neodímio grau térmico **N35SH ou N42H** (suportando até 150 °C na carcaça do motor).

---

## 👥 Autoria e Licença

- **Desenvolvedor:** Guilherme Viana Diniz ([@guilhermevianadiniz](https://github.com/guilhermevianadiniz))
- **Instituição:** SENAI CIMATEC — Salvador, BA
- **Licença:** MIT License (Código aberto para fins educacionais e de pesquisa).
