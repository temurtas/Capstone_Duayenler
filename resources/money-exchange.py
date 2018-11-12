fil=open("expenses.md","r")
data=fil.read().split("|")
data1=[data[i].strip("TL  ") for i in range(11,len(data))]
record=[[data1[i],float(data1[i+2])] for i in range(0,len(data1)-1,5)]

names=[]
for i in range(len(record)):		#finds the names
	names.append( record[i][0] )
	for j in range(len(names)-1):
		if(record[i][0]==names[j]):
			names.pop()
total=[]
asum=0
the_sum=0
for i in range(len(names)):			#calculates total expenses
	for j in range(len(record)):
		if(record[j][0]==names[i]):
			asum+=record[j][1]
	total.append([names[i],asum])
	the_sum+=asum
	asum=0

for i in range(len(total)):			#prints the results
	anet=the_sum/len(total)-total[i][1]
	if(anet>0):
		print(total[i][0]+" should spend "+ "{0:.1f}".format(anet)+" more liras" )
	elif(anet<0):
		print(total[i][0]+" should be given "+ "{0:.1f}".format(-1*anet)+" liras" )
	else:
		print(total[i][0]+" is free to go" )

