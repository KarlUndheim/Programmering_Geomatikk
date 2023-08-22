# Programmering_Geomatikk
Programmering i geomatikk. Oppgave er å klassifisere fartshumper i en punktsky av en lang vei.

Se rapporten som er vedlagt for å få innføring i oppgaven og løsningen. Her vil jeg forklare i hvilke steg koden kjøres for å oppnå resultatet:

1. TerrainFiltering.py
  - input: 0-0.las
  - output: 0-0_clean.las
  
2. noise.py
   - input: 0-0_clean.las
   - output: 0-0_Processed.las

3. detection.py
  - input: 0-0_Processed.las
  - output: TOTAL_BUMPS.las, NOT_BUMPS.las

# Små humper detekteres
4.1 clustering.py
  - input: TOTAL_BUMPS.las
  - output: clustering.las (RESULTAT SMÅ HUMPER)

# Små humper detekteres
4.2 bigBumps.py
  - input: NOT_BUMPS.las
  - output: BIG_BUMPS.las (RESULTAT STORE HUMPER)

