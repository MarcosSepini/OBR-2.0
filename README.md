# 🤖 Temporada 2026


## 🛠️ Tecnologias Utilizadas

| Tecnologia | Finalidade |
| :--------- | :--------- |
| 🤖 LEGO MINDSTORMS EV3 | Controle da locomoção e sensores |
| 🍓 Raspberry Pi 5 | Processamento auxiliar e visão computacional |
| 🐍 MicroPython | Desenvolvimento das aplicações da Raspberry Pi |

---

## 🏅 Resultados

| Etapa | Resultado |
| :---- | :-------: |
| 🥈 Regional | **2º Lugar** |
| 🏆 Estadual | _Aguardando competição..._ |
| 🌎 Nacional | _Aguardando classificação..._ |

 <!--<p align="center">
 <img src="Assets/Vencedor_Regional.jpeg" alt="Medalhas e troféu da OBR 2026" width="30%">
 <img src="images/robo.jpg" alt="Robô da equipe" width="30%">
 <img src="images/equipe.jpg" alt="Equipe AutoBots" width="30%"> 
</p> -->

---

## 📂 Estrutura

```text
2026/
├── Lego/
├── Raspberry/
└── README.md
```

Cada diretório contém o código-fonte e os arquivos relacionados à respectiva plataforma.

---

# 🍓 Raspberry PI 

<img src="https://img.shields.io/badge/Raspberry%20Pi-A22846.svg?style=for-the-badge&logo=Raspberry-Pi&logoColor=white" height="50" alt="OpenCV"  />

## 🔧 Componentes

### ⚡ Eletrônica

| Item | Link |
|------|-------|
| Raspberry Pi 5 | <a href="https://www.mercadolivre.com.br/raspberry-pi-5-4gb/p/MLB34101441?pdp_filters=item_id%3AMLB6253767048&from=gshop&matt_tool=56164162&matt_word=&matt_source=google&matt_campaign_id=22090193744&matt_ad_group_id=194474654154&matt_match_type=&matt_network=g&matt_device=c&matt_creative=792355615410&matt_keyword=&matt_ad_position=&matt_ad_type=pla&matt_merchant_id=735098639&matt_product_id=MLB34101441-product&matt_product_partition_id=2389865440508&matt_target_id=pla-2389865440508&cq_src=google_ads&cq_cmp=22090193744&cq_net=g&cq_plt=gp&cq_med=pla&gad_source=1&gad_campaignid=22090193744&gbraid=0AAAAAD93qcAtt2QvSlvNomh3aQ4qjLvla&gclid=Cj0KCQjwp9vTBhCWARIsANaUrjt9xBKliHJzJ25abmT0-zQpq2bBR9l_lmbBZctvhsnZnKa7gZvfVHsaAiaiEALw_wcB">Mercado Livre</a> |
| Picamera | <a href="aa">aa</a> |
| Picamera | <a href="bb">bb</a> |
| Ponte H BTS7960 | <a href="https://38-3d.co.uk/blogs/blog/using-the-bts7960-with-the-raspberry-pi?srsltid=AfmBOoqtV-J1L4_IH4uIYGy9kC5tw5cp9F2e4O6u4g8vivxoIFbKGx5e">38-3D</a> |
| 4x Motores 130 | <a href="https://www.robocore.net/motor-caixas-de-reducao/motor-dc-3-6v-com-caixa-de-reducao-e-eixo-duplo?srsltid=AfmBOoqbfmLo_w1bh3WgVr3No33KMYh-RDxpnDhVMgovxlrx0kWVJad2iIY">RoboCore</a>  |
| 3x Sensor infravermelho | <a href="https://www.makerhero.com/produto/sensor-de-distancia-a-laser-vl53l0x-de-alta-precisao/?gad_source=1&gad_campaignid=22895934719&gbraid=0AAAAADncekEU-j5QLrGrEshErN1PU7G0c&gclid=Cj0KCQjw3qLSBhDaARIsAFTiVh7LduKHB1k-z4UygFLVAckm7rYoLiJ8eeFlBsGe05bS8QrAckUxGCgaAmi6EALw_wcB">MakerHero</a> |
 | Anel led | <a href="https://www.robocore.net/led/modulo-ws2812-led-enderecavel-12-bits?utm_source=&utm_medium=&utm_campaign=&utm_content=&utm_term=&ad_id=&gad_source=1&gad_campaignid=16517456855&gbraid=0AAAAADzrkI6DLXb_7BhYpGls4zM0LfyFY&gclid=Cj0KCQjw3qLSBhDaARIsAFTiVh56jBZ1OS7513B2JmfQOxeeoQvqSAe06r1EMmnvV61G22QsSyGTfQUaAlh2EALw_wcB">RoboCore</a> |
| Micro Servo SG90 | <a href="https://www.makerhero.com/produto/micro-servo-mg90s-towerpro/">MakerHero</a>  |
| 2x baterias LiPo 2S (7.4 V) | <a href="https://www.mercadolivre.com.br/bat-liion-74v-3300mah-modelo-18650-recarregavel/up/MLBU2933670698?pdp_filters=item_id%3AMLB3946716991&from=gshop&matt_tool=49835134&matt_word=&matt_source=google&matt_campaign_id=22090354217&matt_ad_group_id=192555042614&matt_match_type=&matt_network=g&matt_device=c&matt_creative=799032780940&matt_keyword=&matt_ad_position=&matt_ad_type=pla&matt_merchant_id=745422529&matt_product_id=MLBU2933670698&matt_product_partition_id=2470936289000&matt_target_id=pla-2470936289000&cq_src=google_ads&cq_cmp=22090354217&cq_net=g&cq_plt=gp&cq_med=pla&gad_source=1&gad_campaignid=22090354217&gbraid=0AAAAAD93qcBQK5BEIdZn8sRnZ7RnvC5hE&gclid=Cj0KCQjw3qLSBhDaARIsAFTiVh6EJNq2BSiDGarUj8Sw5QbcHLo_wvWzyMvZCByA8jJEE_DLoIsvf0waAv_hEALw_wcB">Mercado Livre</a> |


### 💻 Software

## Bibliotecas utilizadas

<div align="left">
  <img src="https://img.shields.io/badge/OpenCV-5C3EE8.svg?style=for-the-badge&logo=OpenCV&logoColor=white" height="50" alt="OpenCV"  />
  <img width="5" />
  <img src="https://img.shields.io/badge/NumPy-013243.svg?style=for-the-badge&logo=NumPy&logoColor=white" height="50" alt="Numpy"  />
  <img width="5" />
</div>

<br>

## 🚀 Objetivo

O projeto foi desenvolvido para integrar o **LEGO EV3** e a **Raspberry Pi 5**, explorando o potencial de cada plataforma para obter um robô mais robusto, preciso e eficiente nas provas da OBR.
