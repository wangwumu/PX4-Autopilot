import json

class JsonOutput():

    def __init__(self, groups, board):
        self.json_data = {
            "version": 1,
            "airframes": []
        }

        for group in groups:
            for airframe in group.GetAirframes():
                # check if there is an exclude tag for this airframe
                excluded = False
                for code in airframe.GetArchCodes():
                    if "CONFIG_ARCH_BOARD_{0}".format(code) == board and airframe.GetArchValue(code) == "exclude":
                        excluded = True

                if not excluded:
                    entry = {
                        "id": int(airframe.GetId()),
                        "name": airframe.GetName(),
                        "type": airframe.GetType(),
                        "class": airframe.GetClass(),
                        "maintainer": airframe.GetMaintainer(),
                    }

                    # optional extra fields
                    for code in airframe.GetFieldCodes():
                        entry[code] = airframe.GetFieldValue(code)

                    # actuator outputs
                    outputs = {}
                    for code in airframe.GetOutputCodes():
                        value = airframe.GetOutputValue(code)
                        valstrs = value.split(";")
                        output_entry = {"value": valstrs[0]}
                        for attrib in valstrs[1:]:
                            if ":" in attrib:
                                k, v = attrib.split(":", 1)
                                output_entry[k.strip()] = v.strip()
                        outputs[code] = output_entry
                    entry["outputs"] = outputs

                    self.json_data["airframes"].append(entry)

    def Save(self, filename):
        with open(filename, 'w', encoding="UTF-8") as f:
            json.dump(self.json_data, f, ensure_ascii=False, indent=2, sort_keys=True)
