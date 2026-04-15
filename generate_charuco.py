"""ChArUco 보드 이미지를 생성합니다. A4에 인쇄 후 평평한 판에 붙여 사용하세요."""

import cv2

SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH = 0.04  # 40mm (인쇄 후 실측값으로 수정)
MARKER_LENGTH = 0.03  # 30mm (인쇄 후 실측값으로 수정)


def generate():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, dictionary
    )
    img = board.generateImage((800, 1100))  # A4 비율
    cv2.imwrite("calibration/charuco_board.png", img)
    print("charuco_board.png 생성 완료")
    print(f"설정: {SQUARES_X}x{SQUARES_Y}, 사각형 {SQUARE_LENGTH*1000:.0f}mm, 마커 {MARKER_LENGTH*1000:.0f}mm")
    print("인쇄 후 줄자로 사각형 한 변 길이를 측정하고 SQUARE_LENGTH를 맞춰주세요.")


if __name__ == "__main__":
    generate()
