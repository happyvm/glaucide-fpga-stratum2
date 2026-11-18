import json,sys
d=json.load(open(sys.argv[1]))
n=0
for m in d["modules"].values():
    for c in m["cells"].values():
        if c["type"]=="RAM64M":
            for p in list(c["port_directions"]):
                want="output" if p.startswith("DO") else "input"
                if c["port_directions"][p]!=want: c["port_directions"][p]=want; n+=1
json.dump(d,open(sys.argv[1],"w")); print("fixed",n)
