Index	| Sub-Index	| Typically mapped to	| Type | Description
------|-----------|---------------------|------|-------------
0x2000	| 1-15	| 0x6040	| uint16	| For each motor: controlword to send to motor via PDO
0x2001	| 1-15	| 0x6060	| uint8	  | For each motor: Mode of operation
0x2002	| 1-15	| 0x607A	| int32	  | For each motor: Target position
0x2003	| 1-15	| 0x6081	| uint32	| For each motor: Velocity
0x2004	| 1-15	| 0x6082	| uint32	| For each motor: Acceleration
0x2005	| 1-15	| 0x6083	| uint32	| For each motor: Deceleration
0x2006- |       |         |         | 
0x200F	| 			|         |         | Left empty for further output operations with PDOs
0x2010	| 1-15	| 0x6041  | uint16	| For each motor: statusword received from motor via PDO
