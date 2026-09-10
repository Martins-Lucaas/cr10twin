# Capa de silicone da ponteira D — peças e vazamento

Complemento do `Docs/protocolo_coleta_textura.md` (§3, revisão de 03/09/2026).
A ponteira D **não muda**. O que se fabrica é só a capa e o molde dela.

## Arquivos

| Arquivo (`cad/step/`) | O que é |
|---|---|
| `molde_D.step` | Corpo do molde — a cavidade |
| `tampa_D.step` | Tampa/macho — réplica da cabeça da ponteira, com disco de batente |

Origem local das duas: **Z = 0 é a base do bloco da ponteira D**, que na pilha
montada fica em **Z = 147,70 mm** do flange. O topo do laminado fica em Z = 14,50
(= 162,20 na pilha), que é o TCP atual.

## Geometria da ponteira D que o molde reproduz

Medida no STEP, não estimada:

| Elemento | Cota |
|---|---|
| Bloco da cabeça | 22,4 (X) × 24,88 (Y), altura 11,25 mm, cantos chanfrados 2 × 2 a 45° |
| Ressalto do sensor | 17,4 × 19,88 mm, 2,75 mm de altura, com R2 nas duas bordas do topo no eixo Y |
| Laminado 5×5 | 17 × 19,46 × 0,5 mm, assentado sobre o ressalto |
| Rasgo do flex | 6,4 × 2 mm na face −Y, altura toda do bloco |
| Furo da baioneta | ⌀10,2 × 10,39 mm (encaixa na haste ⌀10 do `touch_tool`) |

O macho reproduz o **contorno convexo**: bloco + ressalto + laminado como um sólido
único (ressalto e laminado fundidos num degrau de 3,25 mm). O rasgo do flex é
deliberadamente **ignorado** — assim o interior da capa fica liso e o rasgo da peça
real vira um canal por onde o flex sai, entre a ponteira e a capa.

## A capa

| Parâmetro | Valor |
|---|---|
| Parede lateral | **2,0 mm** |
| Parede no topo (sobre o laminado) | **1,5 mm** |
| Externo | 26,4 × 28,88 mm, cantos R4, topo arredondado R3 |
| Altura total | 16,0 mm |
| Volume | **4,52 cm³** — misturar ~8 mL de Dragon Skin para ter folga de transbordo |

A parede de 1,5 mm sobre o sensor é o número que importa: abaixo de 1 mm a capa rasga
na bucha e transmite impacto pontual; acima de 2 mm ela vira passa-baixas e apaga a
alta frequência que distingue as texturas.

**Retenção:** por aderência e aperto do próprio silicone sobre o bloco. Não há rasgo
de retenção na ponteira D e não é preciso — a capa é moldada exatamente sobre a
geometria dela.

## O molde

**Concha** (`molde_capaD_concha`)

- Corpo cilíndrico **⌀50 mm**, 22 mm de altura útil.
- Cavidade = contorno externo da capa (26,4 × 28,88 R4), **16 mm de profundidade**.
- Fundo de **6 mm**.
- **Canal de transbordo** no rebordo: rebaixo de **0,6 mm** numa faixa de
  32,4 × 34,88 R7 em volta da boca — mesma solução do `molde_corpo_1` existente.
- Dois **pinos de registro ⌀3 × 4 mm** em (±20, 0), como nos moldes já existentes.

**Macho** (`molde_capaD_macho`)

- Disco **⌀50 × 8 mm** que apoia no rebordo da concha e define a profundidade.
- Réplica da cabeça da ponteira projetando-se **14,5 mm** para dentro da cavidade —
  16,0 de cavidade menos 14,5 de macho = **1,5 mm de parede no topo**, e
  (26,4 − 22,4)/2 = **2,0 mm nas laterais**.
- Dois furos **⌀3,3 × 4,5 mm** para os pinos (folga de 0,3 mm, igual à do
  `molde_tampa`).
- Pega **⌀12 × 10 mm** no topo do disco, para puxar na desmoldagem.

Sem contra-saída em nenhuma das duas peças: a cavidade é um prisma reto e o macho sai
puxando em Z.

## Impressão

- PLA ou PETG, camada **0,15 mm**, paredes 3, preenchimento ≥ 40 %.
- **Imprimir a concha com a boca para cima** (girar 180° no fatiador): a superfície
  da cavidade fica muito melhor sem suporte, e ela é a que dá acabamento à capa.
- O macho imprime com o disco na mesa e a réplica para cima.
- Se quiser acabamento liso na capa, dar uma passada de primer/verniz na cavidade e
  no macho antes do primeiro vazamento.

## Vazamento

1. **Desmoldante obrigatório** na cavidade e no macho (Ease Release 200, ou álcool
   polivinílico). Dragon Skin adere a PLA impresso.
2. Misturar **Dragon Skin 10 Medium** 1:1 em volume, ~8 mL. Mexer 2 min raspando as
   paredes do copo. Desaerar em câmara de vácuo se houver; senão, verter em fio fino
   e alto, que já tira boa parte das bolhas.
3. Encher a cavidade **até a boca**, com a concha na posição de boca para cima.
4. Descer o macho **devagar**, alinhado pelos dois pinos, até o disco assentar no
   rebordo. O excesso sai pelo canal de transbordo de 0,6 mm.
5. Curar segundo a folha do produto (Dragon Skin 10: ~5 h a 23 °C; pós-cura de 4 h a
   65 °C deixa a peça mais estável dimensionalmente).
6. Desmoldar puxando o macho pela pega, depois tirar a capa da concha. Aparar a
   rebarba do transbordo com estilete.

**Inibição:** silicone de platina é envenenado por enxofre, estanho e resinas não
pós-curadas. Nada de luva de látex, massa de modelar ou peça de resina em contato
com a mistura.

## Depois de montada

- A capa acrescenta **1,5 mm** ao TCP quando comprimida a zero, e o contato passa a
  ser a face externa dela. Isso **não** exige mexer no URDF: o `tcp_link` continua na
  face do laminado, e a espessura da capa entra como um deslocamento constante que o
  **contato aprendido** absorve na primeira descida. O que muda de verdade é a
  rigidez `K`, e essa o `_StiffnessEstimator` mede sozinho a cada contato.
- **Re-ensinar a HOME não é necessário** — a ferramenta não mudou de comprimento
  nominal. Mas vale **desmarcar *Home conhecida*** uma vez, para o contato aprendido
  ser refeito com a capa montada em vez de herdar o da ponteira nua.
- Trocar a capa (desgaste, rasgo) muda o contato aprendido de novo. Anotar no diário
  de sessão qual capa está montada e quantos runs ela já levou.
