TCC
===
####### MAPAS #######
Os mapas utilizados no projeto estão todos no diretório "code/bitmap/"

####### Ambiente Dinamico #######
Para a execução deve-se utilizar o comando player e escolher o tipo de ambiente que se deseja submeter, no caso de dinamico acessa-se a pasta dynamic e escolhe algum dos arquivos .cfg para passar por parametro para o comando, como pode ser observado no EX1.</br>
EX1: "player ./dynamic/dynamic.cfg"

####### Ambiente Estatico #######
Outra opção para o comando player é utilizar ambientes estaticos, observado no EX2. Em ambientes estaticos foi feito um exemplo com dois robôs, para isso utilizar o comando apresentado no EX3.</br>
EX2: "player ./static/cave.cfg"</br>
EX3: "player ./static/mapaLivroIATwoRobot.cfg"

####### D* #######
Para a execução do algoritmo no ambiente estatico ou dinamico utilizar o código "codeDstar.py", observe que esse não esta funcionando 100% apresenta alguns erros. No EX4 é apresentado o código para executar o código. È importante salientar que o código utiliza a biblioteca implementada, que esta localizada na pasta "DStarLiteJava" e se for rodar o código utilize mapas pequeno, pois se utilizar mapas muito grandes corre-se o risco do algoritmo não produzir uma resposta. O código do D* esta gerando algumas imagens para fins de depuração do código.</br>
EX4: "python codeDstar.py"

####### A* #######
Para a execução do algoritmo no ambiente estatico utilizar o código "codeAstar.py", a execução desse é similar a do D*, apresentado no EX5. Mas para os demais códigos como: "codeAstarHospital.py", "codeAstarHospital_section.py", "codeAstarLivroIA.py" tem a necessidade de se colocar a porta de conexão, como apresentado no EX6 A porta padrão utilizada foi a 6665, no entanto para o exemplo com dois robôs deve-se utilizar a porta 6665 e 6666, cada uma vai controlar um robô. Caso presica-se fazer alguma alteração no código do A* deve-se examinar o arquivo implementation.py que é onde esta o core dos algoritmos para ambientes estaticos.</br>
EX5: "python codeAstar.py"</br>
EX6: "python codeAstarHospital.py 6665"

Observe que há um link entre alguns ambientes e o código utilizado, isso se dá ao fato de alguns mapa serem muito grandes e outros muito pequenos. </br>

AMBIENTE x CODIGO

hospital2.cfg	x "codeAstarHospital.py", </br>
hospital22.cfg x "codeAstarHospital.py", </br>
hospital_section2.cfg x "codeAstarHospital_section.py", </br>
hospital_section22.cfg x "codeAstarHospital_section.py", </br>
mapaLivroIA.cfg x "codeAstarLivroIA.py", </br>
mapaLivroIATwoRobot.cfg x "codeAstarLivroIA.py"</br>

####### Dijkstra #######
Outro algoritmo que poderia ser utilizado para avaliação é o Dijkstra, ele foi implementado depois da entrega do TCC, por isso não foi documentado no trabalho.
