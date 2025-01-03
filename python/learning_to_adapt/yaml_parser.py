import yaml

def load_yaml(yaml_filename):
    with open(yaml_filename) as fd:
        params = yaml.load(fd, Loader=yaml.FullLoader)
    return params
