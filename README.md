#  NRFBox - CC1101 Sub-GHz Expansion

Este repositorio es un *fork* actualizado del proyecto original nRFBox realizado bajo **cifertech**. Esta versión ha sido adaptada y expandida por **HackNRF24Team** para enfocarse en la investigación y auditoría de protocolos de radiofrecuencia de baja frecuencia (Sub-GHz).

##  Descripción General

La innovación principal de esta modificación consiste en la integración paralela de **tres módulos transceptores CC1101** controlados por un microcontrolador ESP32. Mediante la correcta multiplexación del bus SPI, el sistema es capaz de realizar lecturas en crudo (*Raw Copy*) y la clonación o replicación de señales a distancia. 

Este rediseño transforma la herramienta en un dispositivo capaz de auditar sistemas de seguridad que operan en bandas Sub-GHz (como controles de portones, alarmas y sensores inalámbricos) mediante ataques de repetición (*Replay Attacks*).

##  Características y Modificaciones (HackNRF24Team)

* **Integración Multicanal:** Soporte a nivel de hardware y software para 3 antenas CC1101 simultáneas.
* **Multiplexación SPI:** Lógica de control de pines CSN (Chip Select Not) reescrita para evitar colisiones en el bus de datos entre los tres transceptores.
* **Sub-GHz Raw Copy:** Captura, decodificación y almacenamiento temporal de tramas de radiofrecuencia en bandas bajas.
* **Replay Attack Engine:** Capacidad de retransmitir las señales capturadas con alta fidelidad a mayor distancia.

##  Hardware Adicional
Además del hardware base propuesto en el proyecto original (ESP32, Pantalla OLED SSD1306, botones de navegación), esta expansión requiere:
* 3x Módulos Transceptores RF CC1101 (con sus respectivas antenas).
* Cableado y PCB de expansión para el enrutamiento del bus SPI.

##  Contexto Académico
Este proyecto fue desarrollado como trabajo práctico y de investigación aplicada para la asignatura de **Sistemas de Comunicaciones** del programa de **Ingeniería en Computación**.

**Universidad Nacional Autónoma de México (UNAM)** **Facultad de Ingeniería**

**Equipo de Desarrollo (HackNRF24Team):**
* **Aguilar Chávez Cristian Michael** *(Líder de Proyecto)*
* **Martínez García José Eduardo**
* **Jiménez Reyes Alan Yael**
* **Ortíz Valles Joaquín Rafael**

##  Aviso Legal
*Este hardware y software han sido desarrollados con fines estrictamente académicos, educativos y de investigación en el área de telecomunicaciones y ciberseguridad. Los autores y la institución no se hacen responsables por el mal uso que se le pueda dar a esta herramienta fuera de entornos de prueba controlados.*

##  Licencia y Créditos
El software base de este repositorio se distribuye bajo la **Licencia MIT**. Las modificaciones y expansiones aquí presentadas mantienen la misma licencia abierta.
