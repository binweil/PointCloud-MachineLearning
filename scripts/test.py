import sqlite3

path = "/home/lamy/Desktop/PCD_MachineLearning/data/result"
conn = sqlite3.connect(path)

c = conn.cursor()
table_name = 'minmax'
id_column = 'my_1st_column'
column_name = 'minx'

try:
    c.execute("INSERT INTO minmax (id, minx, miny, minz, maxx,maxy,maxz,name) \
    VALUES ({index},{mx},{my},{mz},{max},{may},{maz},{n})".\
              format(index=1,mx=1,my=1,mz=1,max=2,may=2,maz=2,n=1))
except sqlite3.IntegrityError:
    print('ERROR: ID already exists in PRIMARY KEY column {}'.format(id_column))

conn.commit()
conn.close()
