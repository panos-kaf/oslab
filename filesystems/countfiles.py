dircnt=regcnt=0
with open('inode_table', 'r', encoding='utf-8') as infile:
	for line in infile:
		if line[2]=='4':
			dircnt+=1
		if line[2]=='8':
			regcnt+=1
			
print('found',dircnt,'dirs\nfound',regcnt,'regular files')
