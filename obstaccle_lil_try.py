 # Check if we are near an obstacle if not blue = Avoid
        while Nearobstacle(hownear):
            if area > 5000:
                print("We are near an obstacle but it is blue!")
                break
            if area < 5000:
                print("There is an obstacle in the way! Avoid!!")
                Avoidobstacle()
