import cv2
import numpy as np
import matplotlib.pyplot as plt
import math

Video_file = 'C:\\Users\\bit\\Downloads\\drive_test_01.avi'
cap = cv2.VideoCapture(Video_file)

ym_per_pix = 30 / 480
xm_per_pix = 3.7 / 480

h = 480
w = 640

# frame_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('C:\\Users\\bit\\Desktop\\drive_test_lane_detection.mp4', fourcc, 20.0, frame_size)

def gray_histogram(image):
    h = np.zeros((image.shape[0], 256), dtype=np.uint8)
    hist_item = cv2.calcHist([image],[0],None,[256],[0,256])
    cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
    hist=np.int32(np.around(hist_item))
    for x,y in enumerate(hist):
        cv2.line(h,(x,0+10),(x,y+10),(255,255,255))
    y = np.flipud(h)
    return y

def calibration():
    wc = 10
    hc = 7
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((wc * hc, 3), np.float32)
    objp[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)
    objpoints = []
    imgpoints = []
    file = 'C:\\Users\\bit\\Downloads\\new_test.png'
    src_img = cv2.imread(file)
    check_borad_img = cv2.resize(src_img, dsize = (640, 480), interpolation = cv2.INTER_AREA)
    check_gray = cv2.cvtColor(check_borad_img, cv2.COLOR_BGR2GRAY)
    print(criteria)
    ret, corners = cv2.findChessboardCorners(check_gray, (wc, hc), None)

    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(check_gray, corners, (10, 10), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        check_borad_img = cv2.drawChessboardCorners(check_borad_img, (wc, hc), corners2, ret)
        # cv2.imshow('check_board', check_borad_img)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, check_gray.shape[::-1], None, None)
        height, width = check_borad_img.shape[:2]
        newcameramtx, ROI = cv2.getOptimalNewCameraMatrix(mtx, dist, (width, height), 0.6, (width, height))
        newcameramtx2, ROI2 = cv2.getOptimalNewCameraMatrix(mtx, dist, (width, height), 0.8, (width, height))
        print(mtx, dist, newcameramtx)

    return mtx, dist, newcameramtx

def onChange(pos):
    global img
    r = cv2.getTrackbarPos('R', 'img')
    g = cv2.getTrackbarPos('G', 'img')
    b = cv2.getTrackbarPos('B', 'img')

    img[:] = (b, g, r)
    return b, g, r

def color_filter(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    # cv2.imshow("h", h)
    # cv2.imshow("s", s)
    # cv2.imshow("v", v)
    # cv2.imshow("hsv", hsv)
    # b, g, r = onChange(0)
    # cv2.inRange(단일 채널 이미지, 최솟값, 최댓값)
    v = cv2.inRange(v, 120, 255)

    masked_img = cv2.bitwise_and(image, image, mask = v)
    return masked_img

def roi(image, i):
    x = int(image.shape[1])
    y = int(image.shape[0])

    _shape = [1, 2, 3]

    # 한 붓 그리기
    _shape[0] = np.array([[int(0.15*x), 0], [int(0.8*x), 0], [int(0.8*x), y], [int(0.15*x), y]])  # 전체 roi
    _shape[1] = np.array([[int(0.2*x), y-200], [int(0.8*x), y-200], [int(0.8*x), y], [int(0.2*x), y]])  # dim 0.6
    _shape[2] = np.array([[int(0.1*x), y-200], [int(0.5*x), y-200], [int(0.5*x), y], [int(0.1*x), y]])

    mask = np.zeros_like(image)

    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    cv2.fillPoly(mask, np.int32([_shape[i]]), ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def wrapping(image, num):
    (h, w) = (image.shape[0], image.shape[1])

    source = [1, 2]
    destination = [1, 2]

    # source = np.float32([[0.4*w, 0.45 * h], [0.6 * w, 0.45 * h], [0, h], [w, h]])  # dim 0.4
    # destination = np.float32([[0.18*w, 0], [0.9*w, 0], [0.2*w, h], [0.8*w, h]])  # dim 0.4
    source[0] = np.float32([[0.4 * w, 0.4 * h], [0.6 * w, 0.4 * h], [0, h], [w, h]])  # dim 0.6
    destination[0] = np.float32([[0.27 * w, 0], [0.67*w, 0], [0.27 * w, h], [0.73 * w, h]])  # dim 0.6
    source[1] = np.float32([[0.4 * w, 0.6 * h], [0.6 * w, 0.6 * h], [0, h], [w, h]])  # dim 0.8
    destination[1] = np.float32([[0.25 * w, 0], [0.95*w, 0], [0.27 * w, h], [0.73 * w, h]])  # dim 0.8

    transform_matrix = cv2.getPerspectiveTransform(source[num], destination[num])
    minv = cv2.getPerspectiveTransform(destination[num], source[num])
    _image = cv2.warpPerspective(image, transform_matrix, (w, h))
    return _image, minv

def plothistogram(image, num):
    histogram = np.sum(image[h-200:h, :], axis=0)
    midpoint = [300, 800]
    point = midpoint[num-1]
    # plt.plot(histogram)
    # plt.show()
    return histogram, point

def window_roi(binary_warped, histogram, midpoint, window_img, num):
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))
    max_left = np.argmax(histogram[:midpoint]) ## 왼쪽 선의 가장 왼쪽
    max_right = np.argmax(histogram[midpoint:]) + midpoint  ## 오른쪽 선의 가장 오른쪽
    if max_left < 100:
        max_left = 195
    if max_right < 310:
        max_right = 415

    nwindows = num
    nonzero = binary_warped.nonzero()  ## 선이 있는 부분의 인덱스만 저장
    nonzero_y = np.array(nonzero[0])  ##  선이 있는 부분 y의 인덱스 값
    nonzero_x = np.array(nonzero[1])  ##  선이 있는 부분 x의 인덱스 값
    # np.set_printoptions(threshold=sys.maxsize)
    margin = 30
    minpix = 50
    left_lane = []
    right_lane = []
    color = [0, 255, 0]
    thickness = 2

    # nonzero는 index를 반환한다
    for w in range(nwindows):
        win_y_top = h-(100*(w+1))  ## window 윗부분
        win_y_bottom = h-(100*w)  ## window 아래부분
        hist = np.sum(binary_warped[win_y_top:win_y_bottom, :], axis = 0)
        maxleft = np.argmax(hist[:midpoint])
        maxright = np.argmax(hist[midpoint:]) + midpoint
        if maxleft < 150 or maxleft > 250:
            maxleft = maxright - 230
        if maxright < 380 or maxright > 450:
            maxright = maxleft + 230
        win_xleft_top = maxleft - margin  ## 왼쪽 window 왼쪽 부분
        win_xleft_bottom = maxleft + margin  ## 왼쪽 window 오른쪽 부분
        win_xright_top = maxright - margin  ## 오른쪽 window 왼쪽 부분
        win_xright_bottom = maxright + margin  ## 오른쪽 window 오른쪽 부분
        plt.plot(hist)

        cv2.rectangle(out_img, (win_xleft_top, win_y_top), (win_xleft_bottom, win_y_bottom), color, thickness)
        cv2.rectangle(out_img, (win_xright_top, win_y_top), (win_xright_bottom, win_y_bottom), color, thickness)

        good_left = ((nonzero_y >= win_y_top) & (nonzero_y < win_y_bottom) & (nonzero_x >= win_xleft_top) & (nonzero_x < win_xleft_bottom)).nonzero()[0]
        good_right = ((nonzero_y >= win_y_top) & (nonzero_y < win_y_bottom) & (nonzero_x >= win_xright_top) & (nonzero_x < win_xright_bottom)).nonzero()[0]
        left_lane.append(good_left)
        right_lane.append(good_right)

        if len(good_left) > minpix:
            maxleft = np.int(np.mean(nonzero_x[good_left]))
        if len(good_right) > minpix:
            maxright = np.int(np.mean(nonzero_x[good_right]))

    left_lane = np.concatenate(left_lane)  ## array 1차원으로 합침
    right_lane = np.concatenate(right_lane)

    leftx = nonzero_x[left_lane]
    lefty = nonzero_y[left_lane]
    rightx = nonzero_x[right_lane]
    righty = nonzero_y[right_lane]
    lst = range(100)
    # LEFT
    if len(leftx) < 10 or len(lefty) < 10:
        leftx = np.array([[200] for _ in range(100)])
        lefty = np.array(lst)
    # RIGHT
    if len(rightx) < 10 or len(righty) < 10:
        rightx = np.array([[400] for _ in range(100)])
        righty = np.array(lst)

    # print('left', leftx)
    print(np.mean(leftx))
    # print('right', rightx)
    print(np.mean(rightx))
    print("##############################")

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    ploty = np.linspace(h-200, h, h-200)
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    # lradius = Radius(left_fitx)
    # rradius = Radius(right_fitx)
    # print(lradius, rradius)

    ltx = np.trunc(left_fitx)  ## 소수점 부분 버림
    rtx = np.trunc(right_fitx)  ## 소수점 부분 버림

    cv2.imshow('window', out_img)
    # plt.imshow(window_img)
    # plt.plot(left_fitx, ploty)
    # plt.plot(right_fitx, ploty)
    # plt.xlim(0, 640)
    # plt.ylim(480, 0)
    # plt.show()

    ret = {'left_fitx': ltx, 'right_fitx': rtx, 'ploty': ploty}

    return ret

def Radius(fitx):
    ## Radius 곡률 반경 구하기
    W = math.sqrt((fitx[0] - fitx[-1]) ** 2 + 720 ** 2)
    m = (fitx[0] - fitx[-1]) / 720  # A와 B를 지나는 직선의 기울기 m
    plt.plot((fitx[0] - fitx[-1]) / 2 + fitx[-1], 359, 'ro')  # A와 B를 잇는 직선의 중점
    M = ((fitx[0] - fitx[-1]) / 2 + fitx[-1], 359)
    # print(-1 / m)  # 기울기 m인 직선과 수직인 기울기
    # print(M)
    lean = []
    cy = 0
    for cx in fitx:
        lean.append(abs((-1 / m) - ((cy - M[1]) / (M[0] - cx))))
        cy += 1
    idx = lean.index(min(lean))
    C = [fitx[idx], idx]
    H = math.sqrt((C[0] - M[0]) ** 2 + (C[1] - M[1]) ** 2)
    R = H / 2 + W ** 2 / 8 * H
    pi2me = 0.000264583
    R *= pi2me
    return R

def measure_lane_curvature(ploty, leftx, rightx):

    leftx = leftx[::-1]  # Reverse to match top-to-bottom in y
    rightx = rightx[::-1]  # Reverse to match top-to-bottom in y

    # Choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)

    # Fit new polynomials to x, y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)

    # Calculate the new radii of curvature
    left_curverad  = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters
    # print(left_curverad, 'm', right_curverad, 'm')

    # Decide if it is a left or a right curve
    if leftx[0] - leftx[-1] > 80:
        curve_direction = 'Left Curve'
    elif leftx[-1] - leftx[0] > 80:
        curve_direction = 'Right Curve'
    else:
        curve_direction = 'Straight'

    return (left_curverad + right_curverad) / 2.0, curve_direction

def draw_lane_lines(original_image, warped_image, Minv, draw_info):

    left_fitx = draw_info['left_fitx']
    right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']

    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))
    # plt.plot(pts, ploty)

    mean_x = np.mean((left_fitx, right_fitx), axis=0)
    pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])
    # print(list(pts_mean[0]))

    ## 차선 중심 그래프
    # plt.plot([pts_mean[0][0][0], pts_mean[-1][-1][0]], [480, 0])
    # plt.xlim(0, 640)

    cv2.fillPoly(color_warp, np.int_([pts]), (216, 168, 74))

    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.4, 0)

    return pts_mean, result

mtx, dist, newcameramtx = calibration()

while True:
    retval, src = cap.read()

    if not retval:
        break

    # 원본
    # cv2.imshow('original', src)

    # 색 반전, gray
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    not_gray = ~gray
    # cv2.imshow('reverse', gray)

    # fisheye lense calibration
    img = cv2.undistort(not_gray, mtx, dist)
    img2 = cv2.undistort(not_gray, mtx, dist, None, newcameramtx)
    # cv2.line(img, (100, h), (310, h//2), (0, 255, 0), 3)
    # cv2.line(img, (640, h), (450, h//2), (0, 255, 0), 3)
    # cv2.imshow('calibresult', img)
    cv2.imshow('optimize', img2)
    img_lst = [img, img2]

    # 조감도 wrapped img
    warpped_img, minverse = wrapping(img_lst[1], 0)
    # cv2.imshow('warpping', warpped_img)

    # color filter HSV(빛 의존성 최소화)
    # filtered_img = color_filter(warpped_img)
    # cv2.imshow('filter', filtered_img)

    # gray histogram
    # line = gray_histogram(not_gray)
    # cv2.imshow('gray histogram', line)

    # 조감도 자르기 wrapped img roi
    w_roi_one = roi(warpped_img, 1)
    # cv2.imshow('w_roi', w_roi_one)

    # sobelx = cv2.Sobel(w_roi_one, cv2.CV_64F, 1, 0, ksize = 1)
    # cv2.imshow('sobel', sobelx)

    # 조감도 선 따기 wrapped img threshold
    ret, thresh = cv2.threshold(w_roi_one, 180, 255, cv2.THRESH_BINARY)
    cv2.imshow('threshold', thresh)

    sobelx = cv2.Sobel(thresh, cv2.CV_64F, 1, 0, ksize = 1)
    # cv2.imshow('sobel', sobelx)

    # 선 분포도 조사 histogram
    hist, mpoint = plothistogram(thresh, 1)

    # histogram 기반 window roi 영역
    draw_info = window_roi(thresh, hist, mpoint, warpped_img, 1)

    # 곡률 계산
    # curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx)
    # print(curveRad, curveDir)

    # 원본 이미지에 라인 넣기
    # meanPts, result = draw_lane_lines(img, thresh, minverse, draw_info)
    # cv2.imshow("result", result)

    # 동영상 녹화
    # out.write(result)

    # plt.pause(0.002)

    key = cv2.waitKey(10)
    if key == 27:
        break

# plt.show()
if cap.isOpened():
    cap.release()

cv2.destroyAllWindows()


