import os
import platform
import zipfile
import gdown

def get_assets():
        asset_name = get_asset_name(platform.system())
        print('downloading', asset_name + '.zip')
        system = platform.system().lower()
        gdown.download("https://drive.google.com/uc?id=1RiPz4dk-Ea46R9TjhbWfVkWGyQq2LMtB","MAC.zip", False)
        zip_ref = zipfile.ZipFile('MAC.zip', 'r')
        print('unpacking ...')
        zip_ref.extractall()
        zip_ref.close()
        print("Unpacked !")
    
def get_asset_name(plateform):
    if platform.system().lower() == "darwin" :
        asset_name = "MAC"
    return asset_name


get_assets()