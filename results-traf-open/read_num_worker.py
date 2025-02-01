f = open("random100.txt")
print(f.read().count("Worker")/1000.0)
f.close()

f = open("random200.txt")
print(f.read().count("Worker")/2000.0)
f.close()

f = open("random300.txt")
print(f.read().count("Worker")/3000.0)
f.close()

f = open("random400.txt")
print(f.read().count("Worker")/4000.0)
f.close()

f = open("random500.txt")
print(f.read().count("Worker")/5000.0)
f.close()