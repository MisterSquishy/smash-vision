
*1. Frame-by-frame analyzer:*
- Find characters and their animation states
- Find frame interval analyzed (
- Write to .csv (maybe local array and write off at the end?)

*2. UI, youtube link -> video -> stats -> database*
-	What do we need from user?
-		Names of players, date, tournament (round?)
-	How do we validate this data before putting it in the database?

*3. Derived tables*
-	Primary keys: players, date, tournament/round, game #, frame interval
-	Generated by engine: animation states
-	From this, derive tables:
-	Lowest level: button inputs (->A, etc)
-	Higher: move names (fsmash, etc)
-	Highest: above w/ tech (wavedash, shffl, etc)

*4. Low-level reports:*
-   combos
-	situational decision making

*5. Higher-level reports*
-	Stage control
-	Edgeguarding efficiency
-	Decision trees/Option coverage/Reads
-	Spacing

*6. Metagame reports*
-	What works against certain characters?
-	Matchup records
-	Stage data

*Known limitations:*
-	YouTube: <30 fps
-	DI can't be identified (right?)
-	Fast inputs (worst case: 7 game frames btw youtube frames)

*Progress:*
-   Working on basic object detection, identifying pikachu and fox in game starting positions
