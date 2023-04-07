import pygame as pyg
radii=1
clr=1
screen = pyg.Surface((600, 200))
#Define the rectangles which make the base map
rect_color = (255, 255, 255)
#Define the rectangle which makes the outer border
rectangle1 = pyg.Rect(clr+radii, clr+radii, 600-2*(clr+radii), 200-2*(clr+radii))
screen.fill((255,0,0))
pyg.draw.rect(screen, rect_color, rectangle1)
#Define the rectangle which makes the 2 rectangles
bottom_rect_dim = [(250-radii-clr,200),(265+radii+clr,200),(265+radii+clr,75-radii-clr),(250-radii-clr,75-radii-clr)]
pyg.draw.polygon(screen, (255,0,0),bottom_rect_dim)
top_rect_dim = [(150-radii-clr,0),(165+radii+clr,0),(165+radii+clr,125+radii+clr),(150-radii-clr,125+radii+clr)]
pyg.draw.polygon(screen,(255,0,0),top_rect_dim)

pyg.draw.circle(screen, (255,0,0,),(400,90),50+radii+clr)


white = (255,255,255)
screen_display = pyg.display.set_mode((600, 200)) # Create a screen
screen_display.blit(screen, (0, 0))
pyg.display.update()

# Set the caption of the screen
pyg.display.set_caption('A* Visualization Map')
pyg.display.update()
pyg.time.wait(1)
running=True
while running:
	# for loop through the event queue
	for event in pyg.event.get():
		# Check for QUIT event	
		if event.type == pyg.QUIT:
			running = False

