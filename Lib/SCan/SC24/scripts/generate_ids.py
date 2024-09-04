import json

#512 con dspace_peripheral control
mcb_f=open("./MCB.json")
mcb = json.load(mcb_f)

ecu_ids = mcb["signal_enums"]["ecu_id_Enum"];
for message in mcb["messages"]:
    sender = mcb["messages"][message]["sender"]
    if sender == "BRUSA" :
        continue
    sender_ecu_id = ecu_ids[sender]
    message_id = mcb["messages"][message]["message-id_dec"]
    old_can_id=mcb['messages'][message]["id"]
    new_can_id = message_id*(2**4)+sender_ecu_id;
    print(sender,'[',sender_ecu_id,']',message,'[',message_id,hex(message_id),']',':',old_can_id,hex(old_can_id),'->',new_can_id,hex(new_can_id))
    mcb['messages'][message]["id"]=new_can_id

mcb_fw=open("./MCB_gen.json","w")
json.dump(mcb,mcb_fw,indent=4)
