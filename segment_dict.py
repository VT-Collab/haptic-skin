import pickle

 #enter user, task, and method:"GUI",ROBOT,FLAT
Segment_dict ={(1,1,'GUI'):1,(1,2,'GUI'):1,(1,3,'GUI'):3,
    (1,1,'FLAT'):1,(1,2,'FLAT'):3,(1,3,'FLAT'):1,
    (1,1,'ROBOT'):3,(1,2,'ROBOT'):1,(1,3,'ROBOT'):1,
    
    (2,1,'GUI'):1,(2,2,'GUI'):3,(2,3,'GUI'):1,
    (2,1,'FLAT'):3,(2,2,'FLAT'):3,(2,3,'FLAT'):1,
    (2,1,'ROBOT'):1,(2,2,'ROBOT'):1,(2,3,'ROBOT'):3,
    
    (3,1,'GUI'):1,(3,2,'GUI'):3,(3,3,'GUI'):1,
    (3,1,'FLAT'):3,(3,2,'FLAT'):1,(3,3,'FLAT'):3,
    (3,1,'ROBOT'):3,(3,2,'ROBOT'):3,(3,3,'ROBOT'):1,
    
    (4,1,'GUI'):1,(4,2,'GUI'):1,(4,3,'GUI'):3,
    (4,1,'FLAT'):1,(4,2,'FLAT'):3,(4,3,'FLAT'):1,
    (4,1,'ROBOT'):3,(4,2,'ROBOT'):1,(4,3,'ROBOT'):1,
    
    (5,1,'GUI'):1,(5,2,'GUI'):3,(5,3,'GUI'):1,
    (5,1,'FLAT'):3,(5,2,'FLAT'):3,(5,3,'FLAT'):1,
    (5,1,'ROBOT'):1,(5,2,'ROBOT'):1,(5,3,'ROBOT'):3,
    
    (6,1,'GUI'):1,(6,2,'GUI'):3,(6,3,'GUI'):1,
    (6,1,'FLAT'):3,(6,2,'FLAT'):1,(6,3,'FLAT'):3,
    (6,1,'ROBOT'):3,(6,2,'ROBOT'):3,(6,3,'ROBOT'):1,
    
    (7,1,'GUI'):1,(7,2,'GUI'):1,(7,3,'GUI'):3,
    (7,1,'FLAT'):1,(7,2,'FLAT'):3,(7,3,'FLAT'):1,
    (7,1,'ROBOT'):3,(7,2,'ROBOT'):1,(7,3,'ROBOT'):1,
    
    (8,1,'GUI'):1,(8,2,'GUI'):3,(8,3,'GUI'):1,
    (8,1,'FLAT'):3,(8,2,'FLAT'):3,(8,3,'FLAT'):1,
    (8,1,'ROBOT'):1,(8,2,'ROBOT'):1,(8,3,'ROBOT'):3,
    
    (9,1,'GUI'):1,(9,2,'GUI'):3,(9,3,'GUI'):1,
    (9,1,'FLAT'):3,(9,2,'FLAT'):1,(9,3,'FLAT'):3,
    (9,1,'ROBOT'):3,(9,2,'ROBOT'):3,(9,3,'ROBOT'):1,
    
    (10,1,'GUI'):1,(10,2,'GUI'):1,(10,3,'GUI'):3,
    (10,1,'FLAT'):1,(10,2,'FLAT'):3,(10,3,'FLAT'):1,
    (10,1,'ROBOT'):3,(10,2,'ROBOT'):1,(10,3,'ROBOT'):1,}
filename = 'segments'
pickle.dump(Segment_dict, open(filename, "wb"))
    
    # Segment = Segment_dict{(user,task,method)}
    #method = 'GUI', 'ROBOT', 'FLAT'