# execfile("/Users/Ben/Documents/Kicad/Projects/JNTUB/hardware/scripts/smd_convert.py", locals())

import pcbnew
import math


LIB_SDIY = '/Users/Ben/Documents/Kicad/Projects/JNTUB/hardware/Libraries/SDIY.pretty'

MM = 1000000
INCH = 25.5 * MM
MIL = INCH / 1000

FOOTPRINT_R = 'R_0805_HandSoldering'
FOOTPRINT_R_BIG = 'R_0805_1206'
FOOTPRINT_D = 'D_SOD-323_HandSoldering'
FOOTPRINT_D_PWR = 'D_SOD-123_HandSoldering'
FOOTPRINT_C_CERM = 'C_0805_HandSoldering'
FOOTPRINT_C_ELEC = 'CP_Elec_5x5.3'
FOOTPRINT_REG = 'SOT-89-3'
FOOTPRINT_Q = 'SOT-23_HandSoldering'

RESISTORS = [
    'R1',
    'R3',
    'R7',
    'R9',
    # 'R21',
    # 'R29',
    # 'R15',
    'R17',

    # 'R23',
    'R24',
    'R25',

    'R11',
    'R12',
    'R10',
    'R8',

    'R5',
    'R6',
    'R4',
    'R2',

    'R16',
    # 'R19',
    'R14',
    'R20',

    'R26',
    'R13',
    'R18',

    'R22',
    'R27',
    'R28',
]
RESISTORS_BIG = [
    'R15',
    'R21',
    'R29',
]
RESISTORS_TH = [
    'R19',
    'R23',
]

DIODES = [
    'D1',
    'D2',
    'D3',
    'D4',
    'D5',
]

CAPACITORS_CERM = [
    'C4',
    'C6',
    'C8',
    'C9',
    'C10',
]

CAPACITORS_CERM_BIG = [
    'C12',
]

CAPACITORS_ELEC = [
    'C1',
    'C2',
]

REGS = [
    'U3',
    'U4',
]

POWER_DIODES = [
    'D10',
    'D11',
]

TRANSISTORS = [
    'Q1',
    'Q2',
    'Q3',
]

OFFSETS = {
    'Q2': pcbnew.wxPoint(0, -35 * MIL),
}


board = pcbnew.GetBoard()
mod = board.FindModuleByReference('R11')
pad = mod.FindPadByName('1')
track = board.TracksInNet(board.GetNetcodeFromNetname('GNDREF'))[0]

F_CU = board.GetLayerID('F.Cu')
B_CU = board.GetLayerID('B.Cu')


def Magnitude(wx):
    return math.sqrt(wx.x**2 + wx.y**2)


def Distance(wxa, wxb):
    return Magnitude(wxb - wxa)


def CloseEnough(wxa, wxb):
    return Distance(wxa, wxb) <= (30*MIL)


def Normalize(wx):
    mag = Magnitude(wx)
    return pcbnew.wxPoint(wx.x / mag, wx.y / mag)


def Scale(wx, factor):
    return pcbnew.wxPoint(wx.x * factor, wx.y * factor)


def MidPoint(points):
    x = sum([p.x for p in points]) / len(points)
    y = sum([p.y for p in points]) / len(points)
    return pcbnew.wxPoint(x, y)


def ModuleMidPoint(mod):
    points = [p.GetPosition() for p in mod.Pads()]
    return MidPoint(points)


def GetTracksAtPad(board, pad):
    pos = pad.GetPosition()
    tracks = board.TracksInNet(pad.GetNetCode())
    tracks = [t for t in tracks if track.HitTest(pos)]
    return tracks


def PrintTracks(tracks):
    for track in tracks:
        print('track on layer {} net {} from {},{} to {},{} width {}'.format(
              track.GetLayerName(),
              track.GetNetname(),
              track.GetStart().x, track.GetStart().y,
              track.GetEnd().x, track.GetEnd().y,
              track.GetWidth()))


def CopyText(frm, to):
    to.SetPosition(frm.GetPosition())
    to.SetTextSize(frm.GetTextSize())
    to.SetTextWidth(frm.GetTextWidth())
    to.SetThickness(frm.GetThickness())
    to.SetTextAngle(frm.GetTextAngle())
    to.SetBold(frm.IsBold())
    to.SetItalic(frm.IsItalic())
    to.SetVisible(frm.IsVisible())
    to.SetMirrored(frm.IsMirrored())
    to.SetKeepUpright(frm.IsKeepUpright())
    to.SetLayer(frm.GetLayer())
    to.SetHorizJustify(frm.GetHorizJustify())


def CreateSMTClone(board, mod, footprintLib, footprintName, rot=0, offset=None):
    if offset is None:
        offset = pcbnew.wxPoint(0, 0)
    new_mod = pcbnew.PCB_IO().FootprintLoad(footprintLib, footprintName)
    new_mod.SetPosition(ModuleMidPoint(mod) + offset)
    new_mod.SetOrientation(mod.GetOrientation() + rot)
    new_mod.SetReference(mod.GetReference())
    new_mod.SetValue(mod.GetValue())
    new_mod.SetLocalClearance(mod.GetLocalClearance())
    board.Add(new_mod)
    CopyText(mod.Reference(), new_mod.Reference())
    CopyText(mod.Value(), new_mod.Value())
    for newPad in new_mod.Pads():
        oldPad = mod.FindPadByName(newPad.GetName())
        net = oldPad.GetNet()
        newPad.SetNet(net)
        newPad.SetLocalClearance(oldPad.GetLocalClearance())
    return new_mod


def CreateSMTClone2(board, mod, footprintLib, footprintName):
    new_mod = pcbnew.PCB_IO().FootprintLoad(footprintLib, footprintName)
    new_mod.SetPosition(ModuleMidPoint(mod))
    new_mod.SetOrientation(mod.GetOrientation())
    new_mod.SetReference(mod.GetReference())
    new_mod.SetValue(mod.GetValue())
    new_mod.SetLocalClearance(mod.GetLocalClearance())
    board.Add(new_mod)
    CopyText(mod.Reference(), new_mod.Reference())
    CopyText(mod.Value(), new_mod.Value())
    for newPad in new_mod.Pads():
        oldPad = mod.FindPadByName(newPad.GetName())
        net = oldPad.GetNet()
        newPad.SetNet(net)
        newPad.SetLocalClearance(oldPad.GetLocalClearance())
        # Create via at some offset from the new pad in
        # the direction of the old pad.
        TRACK_WIDTH = int(12 * MIL)
        VIA_DISTANCE = int(30 * MIL)
        direction = Normalize(oldPad.GetPosition() - newPad.GetPosition())
        offset = Scale(direction, VIA_DISTANCE)
        viaPos = newPad.GetPosition() + offset
        via = pcbnew.VIA(board)
        board.Add(via)
        via.SetPosition(viaPos)
        via.SetNet(net)
        via.SetWidth(net.GetNetClass().GetViaDiameter())
        F_CU = board.GetLayerID('F.Cu')
        B_CU = board.GetLayerID('B.Cu')
        via.SetLayerPair(F_CU, B_CU)
        # Create track between new pad and the via.
        track = pcbnew.TRACK(board)
        board.Add(track)
        track.SetStart(newPad.GetPosition())
        track.SetEnd(via.GetPosition())
        track.SetNet(net)
        track.SetWidth(int(12 * MIL))
        track.SetLayer(F_CU)
    return new_mod


def ReplaceModules(board, refs, footprintLib, footprintName, rot=0):
    for ref in refs:
        mod = board.FindModuleByReference(ref)
        if ref in OFFSETS:
            offset = OFFSETS[ref]
        else:
            offset = None
        CreateSMTClone(board, mod, footprintLib, footprintName, rot, offset)
        board.Delete(mod)


def GetNearestPointOnNet(board, refPoint, netcode, layer, selectIt=False):
    nearest = pcbnew.wxPoint(100000 * INCH, 100000 * INCH)
    nearestTrack = None
    tracks = board.TracksInNet(netcode)
    for track in tracks:
        if track.GetLayer() != layer:
            continue
        for point in [track.GetStart(), track.GetEnd()]:
            if Distance(refPoint, point) < Distance(refPoint, nearest):
                nearest = point
                nearestTrack = track
    if selectIt:
        nearestTrack.SetSelected()
    return nearest, nearestTrack


def ConnectModuleToNets(board, mod):
    for pad in mod.Pads():
        padPos = pad.GetPosition()
        padNet = pad.GetNetCode()
        nearestPos, nearestTrack = GetNearestPointOnNet(
            board, padPos, padNet, F_CU
        )
        print('nearest: {},{}'.format(nearestPos.x, nearestPos.y))
        track = pcbnew.TRACK(board)
        board.Add(track)
        track.SetStart(padPos)
        track.SetEnd(nearestPos)
        track.SetNet(board.GetNetsByNetcode()[padNet])
        track.SetWidth(nearestTrack.GetWidth())
        track.SetLayer(F_CU)


def ReplaceAndConnectModules(
    board, refs, footprintLib, footprintName, rot=0, clr=None
):
    for ref in refs:
        mod = board.FindModuleByReference(ref)
        if ref in OFFSETS:
            offset = OFFSETS[ref]
        else:
            offset = None
        newMod = CreateSMTClone(
            board, mod, footprintLib, footprintName, rot, offset
        )
        if clr is not None:
            for pad in newMod.Pads():
                pad.SetLocalClearance(int(clr))
        ConnectModuleToNets(board, newMod)
        board.Delete(mod)


def Run():
    ReplaceModules(board, RESISTORS, LIB_SDIY, FOOTPRINT_R)
    ReplaceModules(board, RESISTORS_BIG, LIB_SDIY, FOOTPRINT_R_BIG)
    ReplaceModules(board, DIODES, LIB_SDIY, FOOTPRINT_D)
    ReplaceModules(board, CAPACITORS_CERM, LIB_SDIY, FOOTPRINT_C_CERM)
    ReplaceAndConnectModules(
        board, CAPACITORS_CERM_BIG, LIB_SDIY, FOOTPRINT_C_CERM
    )
    ReplaceModules(board, CAPACITORS_ELEC, LIB_SDIY, FOOTPRINT_C_ELEC)
    ReplaceAndConnectModules(
        board, REGS, LIB_SDIY, FOOTPRINT_REG, rot=900, clr=12*MIL
    )
    ReplaceModules(board, CAPACITORS_ELEC, LIB_SDIY, FOOTPRINT_C_ELEC)
    ReplaceAndConnectModules(board, POWER_DIODES, LIB_SDIY, FOOTPRINT_D_PWR)
    ReplaceAndConnectModules(
        board, TRANSISTORS, LIB_SDIY, FOOTPRINT_Q, rot=1800
    )
    pcbnew.Refresh()
