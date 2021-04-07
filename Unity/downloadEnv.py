import os
import platform
import zipfile
import gdown

def get_assets():
        asset_name = get_asset_name(platform.system())
        print('downloading', "environments ...")
        gdown.download("https://drive.google.com/uc?id="+asset_name,"env.zip", False)
        zip_ref = zipfile.ZipFile('env.zip', 'r')
        print('unpacking ...')
        zip_ref.extractall()
        zip_ref.close()
        print("Unpacked !")
    
def get_asset_name(plateform):
    if platform.system().lower() == "darwin" :
        asset_name = "1P6l8fkVhz_Rk-bD1p-z7EtAaOShxQ30k"
    if platform.system().lower() == "linux" :
        asset_name = "1D5cRke0A1AQaZ-8G8PfUf13BlLrEgVzl"
    if platform.system().lower() == "windows" :
        asset_name = "1U4sSO2iKldn32C4tZKYVZMeVwQP87nJ8"
    return asset_name



get_assets()

