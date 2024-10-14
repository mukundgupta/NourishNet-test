import requests
import folium

def coordinates(address):
    url = f"https://nominatim.openstreetmap.org/search?q={address}&format=json"

    response = requests.get(url, headers = {'User-Agent' : 'FoodRedistributor/1.0 (mukundgupta2121@gmail.com)'})
    print(response)
    if response.status_code == 200:

        data = response.json()
        print(data)
        print(data[0])
        print(data[0]['lat'])
        lat = data[0]['lat']
        lon = data[0]['lon']
        return lat,lon

def pr_map(coord):
    location = folium.Map(location=coord, zoom_start=16)
    folium.Marker(coord,popup = 'Location').add_to(location)

    location.save('map.html')
    print('location saved to map.html')

c = coordinates("Sri Sathya Sai Vidhya Vihar,indore,india")
pr_map(c)