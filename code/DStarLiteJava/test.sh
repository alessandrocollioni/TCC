rm -r ./txt/*



python DStarLite.py >> txt/python.txt 



javac DStarLite.java -Xlint 

java DStarLite >> txt/java.txt




meld ./txt/java.txt ./txt/python.txt &
