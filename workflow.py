# Start your code below, you can access parameter values as normal list starting from index 1 e.g. i = parameter[1], you can write to output value as normal list starting from index 1 e.g. output[1]= 1+1

obj = parameter[1]["data"]
arrlgt = parameter[1]["count"]["total"] - 1
gas = obj[arrlgt]["Gas detector"]
threshold = 2000

if int(gas) > threshold:
    msgbody='<p>Current gas/smoke value '+gas+' is over threshold.</p><br>'
    output[1]="[Warning] Gas Reading Over Threshold "
    output[2]=msgbody
    output[3]=2

# end your code here #