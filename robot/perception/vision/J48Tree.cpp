#include "J48Tree.hpp"











bool  runJ48Tree(int * attributes) {

int size = attributes[0];
   int width = attributes[1];
      int height = attributes[2];
         int x = attributes[3];
            int y = attributes[4];
               int ratio = attributes[5];
                  int whitePixels = attributes[6];
                     int topDown = attributes[8];
                        int botUp = attributes[9];
                           int leftIn = attributes[10];
                              int rightIn = attributes[11];
                                 int darkestPoint = attributes[12];
                                    int numIslands = attributes[13];

   if (numIslands > 10) { return false; }
   if (numIslands < 2) { return false; }

	if( size <= 10 ) { 
		if( size <= 1 ) { return true; }
	if( size > 1 ) { 
	if( size <= 8 ) { 
		if( leftIn <= 0 ) { return false; }
	if( leftIn > 0 ) { 
	if( rightIn <= 1 ) { 
		if( rightIn <= 0 ) { return false; }
	if( rightIn > 0 ) { 
		if( size > 4 ) { return false; }
	if( size <= 4 ) { 
		if( width > 2 ) { return false; }
		if( width <= 2 ) { return true; }
	}
	}
	}
	if( rightIn > 1 ) { 
	if( width > 3 ) { 
	if( leftIn > 2 ) { 
		if( rightIn > 2 ) { return true; }
		if( rightIn <= 2 ) { return false; }
	}
		if( leftIn <= 2 ) { return false; }
	}
		if( width <= 3 ) { return true; }
	}
	}
	}
	if( size > 8 ) { 
		if( ratio > 63 ) { return false; }
	if( ratio <= 63 ) { 
	if( ratio > 61 ) { 
		if( darkestPoint <= 17 ) { return true; }
	if( darkestPoint > 17 ) { 
		if( darkestPoint <= 37 ) { return false; }
	if( darkestPoint > 37 ) { 
		if( darkestPoint > 55 ) { return false; }
	if( darkestPoint <= 55 ) { 
	if( width <= 6 ) { 
	if( width > 3 ) { 
		if( x > 22 ) { return false; }
	if( x <= 22 ) { 
		if( whitePixels > 641 ) { return true; }
	if( whitePixels <= 641 ) { 
		if( numIslands > 2 ) { return false; }
		if( numIslands <= 2 ) { return true; }
	}
	}
	}
		if( width <= 3 ) { return false; }
	}
		if( width > 6 ) { return false; }
	}
	}
	}
	}
	if( ratio <= 61 ) { 
		if( darkestPoint > 60 ) { return false; }
	if( darkestPoint <= 60 ) { 
	if( width > 3 ) { 
		if( height <= 3 ) { return false; }
	if( height > 3 ) { 
	if( y > 21 ) { 
		if( darkestPoint <= 37 ) { return false; }
		if( darkestPoint > 37 ) { return true; }
	}
	if( y <= 21 ) { 
	if( leftIn > 3 ) { 
		if( leftIn <= 4 ) { return false; }
		if( leftIn > 4 ) { return true; }
	}
	if( leftIn <= 3 ) { 
		if( rightIn <= 0 ) { return false; }
	if( rightIn > 0 ) { 
	if( rightIn <= 1 ) { 
		if( darkestPoint > 44 ) { return true; }
	if( darkestPoint <= 44 ) { 
		if( y > 10 ) { return false; }
		if( y <= 10 ) { return true; }
	}
	}
		if( rightIn > 1 ) { return false; }
	}
	}
	}
	}
	}
		if( width <= 3 ) { return false; }
	}
	}
	}
	}
	}
	}
	if( size > 10 ) { 
	if( ratio > 62 ) { 
	if( height <= 3 ) { 
	if( whitePixels > 660 ) { 
	if( size <= 21 ) { 
	if( width <= 4 ) { 
		if( y > 14 ) { return true; }
		if( y <= 14 ) { return false; }
	}
	if( width > 4 ) { 
	if( y > 8 ) { 
	if( darkestPoint <= 72 ) { 
		if( numIslands <= 1 ) { return true; }
	if( numIslands > 1 ) { 
	if( ratio > 71 ) { 
	if( ratio <= 93 ) { 
	if( topDown > 0 ) { 
	if( topDown > 1 ) { 
		if( rightIn > 3 ) { return false; }
	if( rightIn <= 3 ) { 
		if( rightIn <= 0 ) { return true; }
	if( rightIn > 0 ) { 
		if( leftIn > 2 ) { return false; }
		if( leftIn <= 2 ) { return true; }
	}
	}
	}
		if( topDown <= 1 ) { return false; }
	}
	if( topDown <= 0 ) { 
		if( x <= 7 ) { return false; }
	if( x > 7 ) { 
		if( ratio > 78 ) { return true; }
	if( ratio <= 78 ) { 
	if( x > 11 ) { 
		if( y > 14 ) { return false; }
		if( y <= 14 ) { return true; }
	}
		if( x <= 11 ) { return true; }
	}
	}
	}
	}
		if( ratio > 93 ) { return false; }
	}
		if( ratio <= 71 ) { return false; }
	}
	}
		if( darkestPoint > 72 ) { return false; }
	}
		if( y <= 8 ) { return false; }
	}
	}
	if( size > 21 ) { 
	if( width <= 14 ) { 
		if( topDown > 1 ) { return false; }
	if( topDown <= 1 ) { 
	if( y <= 4 ) { 
		if( ratio > 68 ) { return false; }
		if( ratio <= 68 ) { return true; }
	}
		if( y > 4 ) { return true; }
	}
	}
		if( width > 14 ) { return false; }
	}
	}
	if( whitePixels <= 660 ) { 
	if( width <= 5 ) { 
		if( darkestPoint > 54 ) { return false; }
	if( darkestPoint <= 54 ) { 
		if( x <= 5 ) { return false; }
	if( x > 5 ) { 
		if( y > 20 ) { return false; }
	if( y <= 20 ) { 
		if( x > 18 ) { return false; }
		if( x <= 18 ) { return true; }
	}
	}
	}
	}
	if( width > 5 ) { 
	if( darkestPoint <= 16 ) { 
		if( whitePixels <= 330 ) { return true; }
		if( whitePixels > 330 ) { return false; }
	}
		if( darkestPoint > 16 ) { return false; }
	}
	}
	}
	if( height > 3 ) { 
	if( width > 3 ) { 
	if( darkestPoint <= 52 ) { 
	if( width > 10 ) { 
		if( whitePixels <= 532 ) { return false; }
	if( whitePixels > 532 ) { 
		if( botUp <= 0 ) { return false; }
	if( botUp > 0 ) { 
		if( numIslands <= 1 ) { return true; }
	if( numIslands > 1 ) { 
	if( y > 10 ) { 
	if( height <= 8 ) { 
	if( rightIn <= 12 ) { 
	if( leftIn <= 4 ) { 
		if( whitePixels > 620 ) { return true; }
		if( whitePixels <= 620 ) { return false; }
	}
		if( leftIn > 4 ) { return true; }
	}
		if( rightIn > 12 ) { return true; }
	}
	if( height > 8 ) { 
		if( darkestPoint <= 28 ) { return true; }
	if( darkestPoint > 28 ) { 
	if( rightIn <= 9 ) { 
		if( botUp <= 3 ) { return true; }
		if( botUp > 3 ) { return false; }
	}
		if( rightIn > 9 ) { return true; }
	}
	}
	}
	if( y <= 10 ) { 
		if( rightIn > 14 ) { return true; }
	if( rightIn <= 14 ) { 
	if( leftIn > 11 ) { 
		if( darkestPoint > 39 ) { return true; }
		if( darkestPoint <= 39 ) { return false; }
	}
	if( leftIn <= 11 ) { 
	if( height > 10 ) { 
		if( rightIn > 3 ) { return false; }
		if( rightIn <= 3 ) { return true; }
	}
		if( height <= 10 ) { return false; }
	}
	}
	}
	}
	}
	}
	}
	if( width <= 10 ) { 
	if( whitePixels <= 598 ) { 
	if( x > 0 ) { 
	if( x > 20 ) { 
	if( height > 5 ) { 
	if( ratio <= 67 ) { 
		if( width <= 5 ) { return false; }
		if( width > 5 ) { return true; }
	}
	if( ratio > 67 ) { 
		if( ratio <= 89 ) { return true; }
		if( ratio > 89 ) { return false; }
	}
	}
	if( height <= 5 ) { 
	if( botUp <= 0 ) { 
		if( rightIn > 2 ) { return false; }
		if( rightIn <= 2 ) { return true; }
	}
		if( botUp > 0 ) { return false; }
	}
	}
	if( x <= 20 ) { 
	if( y > 3 ) { 
	if( y > 18 ) { 
	if( width <= 5 ) { 
		if( width <= 4 ) { return false; }
	if( width > 4 ) { 
		if( botUp <= 0 ) { return false; }
	if( botUp > 0 ) { 
		if( size > 14 ) { return true; }
		if( size <= 14 ) { return false; }
	}
	}
	}
	if( width > 5 ) { 
	if( y > 21 ) { 
		if( darkestPoint > 41 ) { return false; }
	if( darkestPoint <= 41 ) { 
		if( x <= 15 ) { return true; }
		if( x > 15 ) { return false; }
	}
	}
		if( y <= 21 ) { return true; }
	}
	}
	if( y <= 18 ) { 
	if( rightIn <= 0 ) { 
		if( ratio > 69 ) { return true; }
	if( ratio <= 69 ) { 
	if( width <= 7 ) { 
	if( width <= 5 ) { 
		if( x <= 6 ) { return false; }
	if( x > 6 ) { 
	if( height > 5 ) { 
		if( topDown <= 3 ) { return false; }
		if( topDown > 3 ) { return true; }
	}
		if( height <= 5 ) { return true; }
	}
	}
		if( width > 5 ) { return true; }
	}
		if( width > 7 ) { return false; }
	}
	}
	if( rightIn > 0 ) { 
	if( ratio > 90 ) { 
		if( size > 48 ) { return false; }
		if( size <= 48 ) { return true; }
	}
	if( ratio <= 90 ) { 
		if( ratio > 63 ) { return true; }
	if( ratio <= 63 ) { 
	if( whitePixels > 534 ) { 
		if( height > 6 ) { return true; }
	if( height <= 6 ) { 
	if( x <= 16 ) { 
		if( numIslands <= 13 ) { return false; }
		if( numIslands > 13 ) { return true; }
	}
		if( x > 16 ) { return true; }
	}
	}
		if( whitePixels <= 534 ) { return true; }
	}
	}
	}
	}
	}
	if( y <= 3 ) { 
	if( topDown > 1 ) { 
	if( height > 7 ) { 
	if( height <= 9 ) { 
		if( whitePixels > 484 ) { return true; }
	if( whitePixels <= 484 ) { 
		if( size <= 46 ) { return true; }
		if( size > 46 ) { return false; }
	}
	}
		if( height > 9 ) { return false; }
	}
		if( height <= 7 ) { return true; }
	}
	if( topDown <= 1 ) { 
		if( botUp <= 0 ) { return false; }
	if( botUp > 0 ) { 
		if( numIslands <= 1 ) { return false; }
	if( numIslands > 1 ) { 
	if( rightIn <= 5 ) { 
	if( rightIn <= 0 ) { 
		if( height > 4 ) { return false; }
		if( height <= 4 ) { return true; }
	}
		if( rightIn > 0 ) { return true; }
	}
		if( rightIn > 5 ) { return false; }
	}
	}
	}
	}
	}
	}
	if( x <= 0 ) { 
	if( topDown <= 6 ) { 
	if( ratio <= 74 ) { 
		if( botUp <= 1 ) { return false; }
	if( botUp > 1 ) { 
	if( botUp <= 2 ) { 
		if( size <= 32 ) { return false; }
		if( size > 32 ) { return true; }
	}
		if( botUp > 2 ) { return false; }
	}
	}
	if( ratio > 74 ) { 
		if( rightIn <= 0 ) { return false; }
	if( rightIn > 0 ) { 
		if( width > 8 ) { return false; }
		if( width <= 8 ) { return true; }
	}
	}
	}
		if( topDown > 6 ) { return true; }
	}
	}
	if( whitePixels > 598 ) { 
	if( y > 9 ) { 
	if( x > 0 ) { 
	if( darkestPoint <= 37 ) { 
	if( width > 8 ) { 
		if( y > 14 ) { return false; }
		if( y <= 14 ) { return true; }
	}
	if( width <= 8 ) { 
		if( y > 20 ) { return false; }
	if( y <= 20 ) { 
	if( ratio > 68 ) { 
	if( botUp <= 0 ) { 
		if( leftIn > 2 ) { return false; }
	if( leftIn <= 2 ) { 
		if( leftIn <= 1 ) { return true; }
	if( leftIn > 1 ) { 
		if( y > 15 ) { return true; }
		if( y <= 15 ) { return false; }
	}
	}
	}
	if( botUp > 0 ) { 
	if( height > 4 ) { 
	if( x > 12 ) { 
		if( darkestPoint <= 23 ) { return false; }
		if( darkestPoint > 23 ) { return true; }
	}
		if( x <= 12 ) { return true; }
	}
	if( height <= 4 ) { 
		if( width <= 4 ) { return true; }
		if( width > 4 ) { return false; }
	}
	}
	}
	if( ratio <= 68 ) { 
	if( numIslands <= 13 ) { 
		if( size <= 36 ) { return false; }
	if( size > 36 ) { 
		if( botUp > 6 ) { return false; }
		if( botUp <= 6 ) { return true; }
	}
	}
		if( numIslands > 13 ) { return true; }
	}
	}
	}
	}
	if( darkestPoint > 37 ) { 
	if( ratio > 68 ) { 
	if( whitePixels <= 666 ) { 
	if( y > 21 ) { 
	if( width <= 5 ) { 
		if( x > 10 ) { return false; }
		if( x <= 10 ) { return true; }
	}
		if( width > 5 ) { return true; }
	}
	if( y <= 21 ) { 
	if( width <= 4 ) { 
	if( height <= 8 ) { 
	if( botUp <= 2 ) { 
	if( whitePixels > 615 ) { 
		if( topDown <= 3 ) { return true; }
		if( topDown > 3 ) { return false; }
	}
	if( whitePixels <= 615 ) { 
		if( x <= 16 ) { return false; }
		if( x > 16 ) { return true; }
	}
	}
	if( botUp > 2 ) { 
		if( y <= 17 ) { return true; }
		if( y > 17 ) { return false; }
	}
	}
		if( height > 8 ) { return false; }
	}
	if( width > 4 ) { 
	if( size > 15 ) { 
	if( rightIn <= 0 ) { 
		if( width <= 6 ) { return true; }
	if( width > 6 ) { 
		if( topDown > 4 ) { return true; }
		if( topDown <= 4 ) { return false; }
	}
	}
		if( rightIn > 0 ) { return true; }
	}
	if( size <= 15 ) { 
		if( size > 14 ) { return true; }
	if( size <= 14 ) { 
		if( whitePixels > 628 ) { return false; }
		if( whitePixels <= 628 ) { return true; }
	}
	}
	}
	}
	}
		if( whitePixels > 666 ) { return true; }
	}
	if( ratio <= 68 ) { 
	if( numIslands > 3 ) { 
	if( botUp <= 0 ) { 
		if( numIslands > 7 ) { return false; }
	if( numIslands <= 7 ) { 
		if( ratio <= 64 ) { return false; }
	if( ratio > 64 ) { 
		if( x > 8 ) { return true; }
		if( x <= 8 ) { return false; }
	}
	}
	}
	if( botUp > 0 ) { 
	if( x <= 14 ) { 
		if( darkestPoint <= 43 ) { return true; }
	if( darkestPoint > 43 ) { 
		if( numIslands > 7 ) { return false; }
	if( numIslands <= 7 ) { 
		if( rightIn <= 0 ) { return false; }
	if( rightIn > 0 ) { 
		if( topDown <= 6 ) { return true; }
	if( topDown > 6 ) { 
		if( rightIn > 6 ) { return true; }
		if( rightIn <= 6 ) { return false; }
	}
	}
	}
	}
	}
	if( x > 14 ) { 
	if( height <= 8 ) { 
	if( width <= 7 ) { 
		if( y > 20 ) { return false; }
	if( y <= 20 ) { 
		if( whitePixels > 649 ) { return true; }
	if( whitePixels <= 649 ) { 
		if( size > 23 ) { return true; }
	if( size <= 23 ) { 
		if( size > 14 ) { return false; }
		if( size <= 14 ) { return true; }
	}
	}
	}
	}
		if( width > 7 ) { return false; }
	}
		if( height > 8 ) { return false; }
	}
	}
	}
	if( numIslands <= 3 ) { 
		if( whitePixels > 660 ) { return true; }
	if( whitePixels <= 660 ) { 
	if( numIslands <= 1 ) { 
		if( width <= 6 ) { return false; }
		if( width > 6 ) { return true; }
	}
	if( numIslands > 1 ) { 
	if( rightIn > 3 ) { 
		if( y > 14 ) { return false; }
		if( y <= 14 ) { return true; }
	}
	if( rightIn <= 3 ) { 
	if( size <= 12 ) { 
		if( y <= 16 ) { return true; }
		if( y > 16 ) { return false; }
	}
		if( size > 12 ) { return true; }
	}
	}
	}
	}
	}
	}
	}
	if( x <= 0 ) { 
		if( topDown > 1 ) { return true; }
	if( topDown <= 1 ) { 
		if( ratio <= 85 ) { return false; }
		if( ratio > 85 ) { return true; }
	}
	}
	}
	if( y <= 9 ) { 
		if( whitePixels > 652 ) { return false; }
	if( whitePixels <= 652 ) { 
	if( numIslands > 2 ) { 
		if( botUp <= 0 ) { return false; }
	if( botUp > 0 ) { 
	if( x > 8 ) { 
		if( width <= 5 ) { return false; }
	if( width > 5 ) { 
	if( leftIn > 2 ) { 
		if( height <= 9 ) { return false; }
		if( height > 9 ) { return true; }
	}
	if( leftIn <= 2 ) { 
	if( y > 1 ) { 
		if( size > 23 ) { return true; }
		if( size <= 23 ) { return false; }
	}
		if( y <= 1 ) { return false; }
	}
	}
	}
		if( x <= 8 ) { return true; }
	}
	}
	if( numIslands <= 2 ) { 
	if( height > 5 ) { 
		if( leftIn <= 0 ) { return false; }
	if( leftIn > 0 ) { 
	if( y <= 4 ) { 
		if( size > 59 ) { return true; }
		if( size <= 59 ) { return false; }
	}
		if( y > 4 ) { return true; }
	}
	}
		if( height <= 5 ) { return true; }
	}
	}
	}
	}
	}
	}
	if( darkestPoint > 52 ) { 
	if( whitePixels > 642 ) { 
	if( darkestPoint > 68 ) { 
	if( whitePixels > 684 ) { 
		if( y > 20 ) { return false; }
		if( y <= 20 ) { return true; }
	}
		if( whitePixels <= 684 ) { return false; }
	}
	if( darkestPoint <= 68 ) { 
	if( y <= 16 ) { 
	if( numIslands <= 5 ) { 
	if( y > 10 ) { 
		if( height > 7 ) { return false; }
	if( height <= 7 ) { 
		if( x > 22 ) { return false; }
	if( x <= 22 ) { 
	if( width <= 17 ) { 
		if( topDown > 1 ) { return true; }
	if( topDown <= 1 ) { 
	if( height > 4 ) { 
		if( size <= 24 ) { return false; }
		if( size > 24 ) { return true; }
	}
		if( height <= 4 ) { return true; }
	}
	}
		if( width > 17 ) { return false; }
	}
	}
	}
	if( y <= 10 ) { 
		if( width <= 7 ) { return false; }
	if( width > 7 ) { 
	if( numIslands > 2 ) { 
		if( size <= 38 ) { return true; }
		if( size > 38 ) { return false; }
	}
		if( numIslands <= 2 ) { return true; }
	}
	}
	}
		if( numIslands > 5 ) { return false; }
	}
	if( y > 16 ) { 
		if( numIslands <= 5 ) { return true; }
	if( numIslands > 5 ) { 
		if( y > 18 ) { return true; }
		if( y <= 18 ) { return false; }
	}
	}
	}
	}
	if( whitePixels <= 642 ) { 
		if( darkestPoint > 54 ) { return false; }
	if( darkestPoint <= 54 ) { 
		if( botUp <= 0 ) { return false; }
	if( botUp > 0 ) { 
		if( x > 19 ) { return false; }
	if( x <= 19 ) { 
		if( height > 11 ) { return false; }
	if( height <= 11 ) { 
	if( topDown <= 2 ) { 
		if( height > 4 ) { return true; }
		if( height <= 4 ) { return false; }
	}
		if( topDown > 2 ) { return true; }
	}
	}
	}
	}
	}
	}
	}
	if( width <= 3 ) { 
	if( y <= 17 ) { 
		if( height > 5 ) { return false; }
	if( height <= 5 ) { 
	if( x > 1 ) { 
	if( botUp <= 1 ) { 
	if( topDown <= 2 ) { 
		if( y <= 5 ) { return false; }
	if( y > 5 ) { 
	if( height > 4 ) { 
	if( y > 9 ) { 
	if( size <= 12 ) { 
	if( numIslands > 2 ) { 
		if( size <= 11 ) { return false; }
	if( size > 11 ) { 
		if( x > 20 ) { return false; }
		if( x <= 20 ) { return true; }
	}
	}
		if( numIslands <= 2 ) { return true; }
	}
		if( size > 12 ) { return false; }
	}
		if( y <= 9 ) { return true; }
	}
	if( height <= 4 ) { 
		if( rightIn <= 0 ) { return true; }
		if( rightIn > 0 ) { return false; }
	}
	}
	}
		if( topDown > 2 ) { return true; }
	}
		if( botUp > 1 ) { return false; }
	}
		if( x <= 1 ) { return false; }
	}
	}
	if( y > 17 ) { 
	if( whitePixels > 677 ) { 
		if( numIslands <= 4 ) { return true; }
	if( numIslands > 4 ) { 
		if( numIslands <= 5 ) { return true; }
		if( numIslands > 5 ) { return false; }
	}
	}
	if( whitePixels <= 677 ) { 
		if( y > 18 ) { return false; }
	if( y <= 18 ) { 
		if( width > 2 ) { return true; }
		if( width <= 2 ) { return false; }
	}
	}
	}
	}
	}
	}
	if( ratio <= 62 ) { 
	if( ratio <= 57 ) { 
	if( whitePixels <= 680 ) { 
	if( whitePixels <= 343 ) { 
		if( darkestPoint > 26 ) { return false; }
	if( darkestPoint <= 26 ) { 
		if( leftIn > 3 ) { return false; }
	if( leftIn <= 3 ) { 
	if( whitePixels <= 295 ) { 
		if( ratio > 55 ) { return true; }
		if( ratio <= 55 ) { return false; }
	}
		if( whitePixels > 295 ) { return true; }
	}
	}
	}
	if( whitePixels > 343 ) { 
	if( ratio > 54 ) { 
		if( botUp <= 0 ) { return false; }
	if( botUp > 0 ) { 
	if( rightIn <= 9 ) { 
		if( whitePixels > 570 ) { return false; }
	if( whitePixels <= 570 ) { 
	if( darkestPoint <= 51 ) { 
		if( rightIn <= 0 ) { return false; }
	if( rightIn > 0 ) { 
	if( botUp > 4 ) { 
		if( height <= 8 ) { return true; }
		if( height > 8 ) { return false; }
	}
	if( botUp <= 4 ) { 
	if( rightIn <= 4 ) { 
	if( topDown > 1 ) { 
	if( height > 7 ) { 
		if( leftIn > 10 ) { return true; }
		if( leftIn <= 10 ) { return false; }
	}
	if( height <= 7 ) { 
		if( x > 20 ) { return false; }
		if( x <= 20 ) { return true; }
	}
	}
		if( topDown <= 1 ) { return false; }
	}
		if( rightIn > 4 ) { return false; }
	}
	}
	}
		if( darkestPoint > 51 ) { return false; }
	}
	}
	if( rightIn > 9 ) { 
		if( numIslands > 6 ) { return false; }
		if( numIslands <= 6 ) { return true; }
	}
	}
	}
	if( ratio <= 54 ) { 
	if( height > 5 ) { 
	if( topDown > 12 ) { 
		if( y > 19 ) { return true; }
		if( y <= 19 ) { return false; }
	}
		if( topDown <= 12 ) { return false; }
	}
	if( height <= 5 ) { 
		if( darkestPoint > 57 ) { return false; }
	if( darkestPoint <= 57 ) { 
		if( botUp <= 1 ) { return false; }
	if( botUp > 1 ) { 
	if( darkestPoint > 41 ) { 
	if( topDown <= 3 ) { 
		if( darkestPoint <= 53 ) { return false; }
	if( darkestPoint > 53 ) { 
		if( botUp <= 2 ) { return true; }
	if( botUp > 2 ) { 
		if( y <= 13 ) { return false; }
		if( y > 13 ) { return true; }
	}
	}
	}
		if( topDown > 3 ) { return true; }
	}
		if( darkestPoint <= 41 ) { return false; }
	}
	}
	}
	}
	}
	}
	if( whitePixels > 680 ) { 
		if( leftIn > 3 ) { return false; }
	if( leftIn <= 3 ) { 
		if( height > 7 ) { return false; }
	if( height <= 7 ) { 
	if( x > 1 ) { 
	if( darkestPoint > 44 ) { 
	if( darkestPoint <= 73 ) { 
	if( numIslands > 3 ) { 
	if( size <= 18 ) { 
		if( whitePixels <= 681 ) { return true; }
		if( whitePixels > 681 ) { return false; }
	}
		if( size > 18 ) { return true; }
	}
	if( numIslands <= 3 ) { 
	if( ratio > 46 ) { 
		if( height > 5 ) { return true; }
	if( height <= 5 ) { 
	if( rightIn <= 0 ) { 
		if( width > 8 ) { return true; }
		if( width <= 8 ) { return false; }
	}
	if( rightIn > 0 ) { 
		if( leftIn > 2 ) { return true; }
	if( leftIn <= 2 ) { 
	if( leftIn <= 0 ) { 
		if( ratio > 51 ) { return true; }
	if( ratio <= 51 ) { 
		if( x > 12 ) { return false; }
		if( x <= 12 ) { return true; }
	}
	}
	if( leftIn > 0 ) { 
	if( height > 4 ) { 
	if( size <= 13 ) { 
		if( numIslands > 2 ) { return false; }
		if( numIslands <= 2 ) { return true; }
	}
		if( size > 13 ) { return false; }
	}
		if( height <= 4 ) { return false; }
	}
	}
	}
	}
	}
	if( ratio <= 46 ) { 
		if( x > 9 ) { return false; }
		if( x <= 9 ) { return true; }
	}
	}
	}
		if( darkestPoint > 73 ) { return false; }
	}
		if( darkestPoint <= 44 ) { return false; }
	}
		if( x <= 1 ) { return false; }
	}
	}
	}
	}
	if( ratio > 57 ) { 
	if( botUp <= 0 ) { 
		if( leftIn <= 1 ) { return false; }
	if( leftIn > 1 ) { 
		if( rightIn > 3 ) { return false; }
	if( rightIn <= 3 ) { 
		if( topDown > 5 ) { return false; }
	if( topDown <= 5 ) { 
	if( rightIn <= 1 ) { 
	if( y > 14 ) { 
		if( whitePixels <= 665 ) { return false; }
	if( whitePixels > 665 ) { 
		if( size > 15 ) { return false; }
		if( size <= 15 ) { return true; }
	}
	}
	if( y <= 14 ) { 
	if( numIslands <= 5 ) { 
		if( numIslands <= 1 ) { return false; }
	if( numIslands > 1 ) { 
	if( rightIn <= 0 ) { 
		if( width <= 5 ) { return true; }
		if( width > 5 ) { return false; }
	}
		if( rightIn > 0 ) { return true; }
	}
	}
		if( numIslands > 5 ) { return false; }
	}
	}
	if( rightIn > 1 ) { 
	if( width <= 7 ) { 
		if( numIslands > 3 ) { return false; }
		if( numIslands <= 3 ) { return true; }
	}
		if( width > 7 ) { return true; }
	}
	}
	}
	}
	}
	if( botUp > 0 ) { 
	if( darkestPoint > 62 ) { 
	if( numIslands <= 1 ) { 
		if( topDown <= 3 ) { return false; }
		if( topDown > 3 ) { return true; }
	}
		if( numIslands > 1 ) { return false; }
	}
	if( darkestPoint <= 62 ) { 
	if( rightIn <= 0 ) { 
		if( leftIn <= 1 ) { return false; }
	if( leftIn > 1 ) { 
	if( height > 5 ) { 
		if( leftIn > 15 ) { return true; }
	if( leftIn <= 15 ) { 
		if( x > 8 ) { return false; }
	if( x <= 8 ) { 
		if( numIslands <= 1 ) { return true; }
		if( numIslands > 1 ) { return false; }
	}
	}
	}
	if( height <= 5 ) { 
		if( ratio > 61 ) { return true; }
	if( ratio <= 61 ) { 
		if( ratio > 60 ) { return false; }
	if( ratio <= 60 ) { 
		if( size > 17 ) { return false; }
	if( size <= 17 ) { 
		if( topDown <= 2 ) { return true; }
	if( topDown > 2 ) { 
		if( botUp <= 2 ) { return false; }
		if( botUp > 2 ) { return true; }
	}
	}
	}
	}
	}
	}
	}
	if( rightIn > 0 ) { 
	if( height > 4 ) { 
	if( topDown > 1 ) { 
	if( whitePixels <= 601 ) { 
	if( width > 10 ) { 
		if( width <= 16 ) { return false; }
		if( width > 16 ) { return true; }
	}
	if( width <= 10 ) { 
	if( rightIn <= 1 ) { 
		if( width <= 4 ) { return false; }
	if( width > 4 ) { 
		if( size <= 30 ) { return true; }
		if( size > 30 ) { return false; }
	}
	}
		if( rightIn > 1 ) { return true; }
	}
	}
	if( whitePixels > 601 ) { 
	if( numIslands <= 1 ) { 
		if( darkestPoint > 30 ) { return true; }
		if( darkestPoint <= 30 ) { return false; }
	}
	if( numIslands > 1 ) { 
	if( width <= 7 ) { 
	if( width <= 4 ) { 
	if( whitePixels > 676 ) { 
		if( size > 14 ) { return false; }
		if( size <= 14 ) { return true; }
	}
		if( whitePixels <= 676 ) { return false; }
	}
	if( width > 4 ) { 
	if( y > 2 ) { 
		if( topDown <= 2 ) { return true; }
	if( topDown > 2 ) { 
	if( rightIn <= 4 ) { 
	if( width <= 6 ) { 
		if( height > 7 ) { return false; }
	if( height <= 7 ) { 
		if( darkestPoint <= 37 ) { return false; }
	if( darkestPoint > 37 ) { 
	if( whitePixels <= 678 ) { 
		if( x <= 16 ) { return true; }
	if( x > 16 ) { 
		if( leftIn <= 1 ) { return false; }
		if( leftIn > 1 ) { return true; }
	}
	}
		if( whitePixels > 678 ) { return false; }
	}
	}
	}
		if( width > 6 ) { return true; }
	}
		if( rightIn > 4 ) { return false; }
	}
	}
		if( y <= 2 ) { return false; }
	}
	}
	if( width > 7 ) { 
	if( x <= 7 ) { 
		if( topDown > 4 ) { return false; }
	if( topDown <= 4 ) { 
		if( size <= 33 ) { return false; }
		if( size > 33 ) { return true; }
	}
	}
		if( x > 7 ) { return false; }
	}
	}
	}
	}
	if( topDown <= 1 ) { 
	if( whitePixels <= 688 ) { 
		if( rightIn <= 12 ) { return false; }
		if( rightIn > 12 ) { return true; }
	}
		if( whitePixels > 688 ) { return true; }
	}
	}
	if( height <= 4 ) { 
	if( numIslands <= 4 ) { 
	if( botUp <= 1 ) { 
		if( darkestPoint <= 53 ) { return false; }
		if( darkestPoint > 53 ) { return true; }
	}
	if( botUp > 1 ) { 
		if( leftIn > 2 ) { return false; }
	if( leftIn <= 2 ) { 
		if( x > 18 ) { return false; }
	if( x <= 18 ) { 
	if( height <= 3 ) { 
		if( whitePixels <= 665 ) { return false; }
		if( whitePixels > 665 ) { return true; }
	}
		if( height > 3 ) { return true; }
	}
	}
	}
	}
	if( numIslands > 4 ) { 
		if( whitePixels <= 392 ) { return true; }
		if( whitePixels > 392 ) { return false; }
	}
	}
	}
	}
	}
	}
	}
	}
return false;
}

