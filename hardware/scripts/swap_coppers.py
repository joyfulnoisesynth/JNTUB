# execfile("/Users/Ben/Documents/Kicad/Projects/JNTUB/hardware/swap_coppers.py", locals())

import pcbnew

board = pcbnew.GetBoard()

F_CU = board.GetLayerID('F.Cu')
B_CU = board.GetLayerID('B.Cu')

track = board.TracksInNet(board.GetNetcodeFromNetname('GNDREF'))[0]

def FlipCoppers(board):
    for netcode, net in board.GetNetsByNetcode().items():
        for track in board.TracksInNet(netcode):
            if track.GetLayer() == F_CU:
                track.SetLayer(B_CU)
            elif track.GetLayer() == B_CU:
                track.SetLayer(F_CU)
    for zone in board.Zones():
        if zone.GetLayer() == F_CU:
            zone.SetLayer(B_CU)
        elif track.GetLayer() == B_CU:
            zone.SetLayer(F_CU)
    pcbnew.Refresh()
