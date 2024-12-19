import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import yaml

# 설정 파라미터
WORLD_FILE = 'car.world'  # .world 파일 경로
MAP_SIZE = 1000            # 맵의 크기 (격자 셀 수)
MAP_RESOLUTION = 0.1       # 각 셀의 크기 (미터/셀)
MAP_ORIGIN = (-50, -50)    # 맵의 원점 (미터)

def parse_sdf(world_file):
    tree = ET.parse(world_file)
    root = tree.getroot()
    
    # 모델 정보를 저장할 리스트
    models = []
    for include in root.findall('.//include'):
        uri = include.find('uri').text if include.find('uri') is not None else ''
        pose = include.find('pose')
        if pose is not None:
            pose_values = list(map(float, pose.text.strip().split()))
            position = pose_values[:3]      # x, y, z
            orientation = pose_values[3:]   # roll, pitch, yaw
            models.append({
                'uri': uri,
                'position': position,
                'orientation': orientation
            })
    return models

def generate_occupancy_grid(models, map_size, resolution, origin):
    grid = np.zeros((map_size, map_size), dtype=np.int8)  # 0: 자유, 1: 점유
    
    # 월드 좌표를 격자 인덱스로 변환하는 함수
    def world_to_grid(x, y):
        grid_x = int((x - origin[0]) / resolution)
        grid_y = int((y - origin[1]) / resolution)
        if 0 <= grid_x < map_size and 0 <= grid_y < map_size:
            return grid_x, grid_y
        else:
            return None, None
    
    for model in models:
        uri = model['uri']
        x, y, z = model['position']
        
        grid_x, grid_y = world_to_grid(x, y)
        if grid_x is not None and grid_y is not None:
            # 모델 종류에 따라 점유 여부 결정
            if ('house' in uri) or ('lamp_post' in uri) or ('stop_sign' in uri) or ('gas_station' in uri) or ('fast_food' in uri):
                grid[grid_y, grid_x] = 1  # 점유
            elif ('road' in uri) or ('stopW' in uri):
                grid[grid_y, grid_x] = 0  # 자유 (기본값이므로 생략 가능)
            # 필요에 따라 추가 조건을 설정하세요.
    
    return grid

def save_map_as_image(grid, filename='map.png'):
    plt.imshow(grid, cmap='gray', origin='lower')
    plt.title('점유 격자 맵')
    plt.xlabel('X 축')
    plt.ylabel('Y 축')
    plt.savefig(filename)
    plt.close()
    print(f"맵 이미지가 {filename}으로 저장되었습니다.")

def save_map_as_yaml(grid, filename='map.yaml', image_filename='map.png'):
    resolution = MAP_RESOLUTION
    origin = MAP_ORIGIN
    height, width = grid.shape
    # 이미지 저장
    save_map_as_image(grid, image_filename)
    data = {
        'image': image_filename,
        'resolution': resolution,
        'origin': [origin[0], origin[1], 0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    with open(filename, 'w') as file:
        yaml.dump(data, file)
    print(f"맵 정보가 {filename}으로 저장되었습니다.")

def main():
    models = parse_sdf(WORLD_FILE)
    grid = generate_occupancy_grid(models, MAP_SIZE, MAP_RESOLUTION, MAP_ORIGIN)
    save_map_as_yaml(grid)

if __name__ == "__main__":
    main()
