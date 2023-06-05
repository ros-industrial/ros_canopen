import yaml
import argparse
from dataclasses import dataclass
from dcfgen.cli import Master, Slave


def main():
    parser = argparse.ArgumentParser(
        description="Expands the CANopen bus.yml for further processing."
    )
    parser.add_argument("--input-file", type=str, help="The name of the input file", required=True)
    parser.add_argument(
        "--output-file", type=str, help="The name of the output file", required=True
    )
    args = parser.parse_args()

    with open(args.input_file) as input:
        cfg = yaml.load(input, yaml.FullLoader)

    master = {}
    if "master" not in cfg:
        print("Found no master.")
        return
    else:
        master = cfg["master"]

    nodes = {}
    if "nodes" not in cfg:
        print("Found no nodes entry.")
        return
    else:
        nodes = cfg["nodes"]

    options = {}
    if "options" in cfg:
        options = cfg["options"]

    defaults = {}
    if "defaults" in cfg:
        defaults = cfg["defaults"]

    # add defaults to each node
    for node_name in nodes:
        for entry_name in defaults:
            nodes[node_name][entry_name] = defaults[entry_name]

    modified_file = {}
    modified_file["options"] = options
    # modified_file["defaults"] = defaults
    modified_file["master"] = master
    for node_name in nodes:
        modified_file[node_name] = nodes[node_name]

    with open(args.output_file, mode="wt") as output:
        yaml.dump(modified_file, output)

    return


if __name__ == "__main__":
    main()
