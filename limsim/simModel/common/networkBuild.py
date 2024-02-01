"""
Author: Licheng Wen, Zhaotong Luo
Copyright (c) 2023 by PJLab, All Rights Reserved. 
"""

# Preprocess for scenario generation
# Get each element's shape from `.net.xml`
# Get network topology from `.net.xml`

# for: NetworkBuild with Frenet
from __future__ import annotations
from utils.simBase import CoordTF, deduceEdge
from utils.cubic_spline import Spline2D
from utils.roadgraph import Junction, Edge, NormalLane, OVERLAP_DISTANCE, JunctionLane, TlLogic
from queue import Queue
import sqlite3
from threading import Thread
import numpy as np
import xml.etree.ElementTree as ET
from rich import print
from datetime import datetime
from PySide6.QtCore import Signal, QPointF
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QWidget
from utils.simBase import graphicsItem

class geoHash:
    def __init__(self, id: tuple[int]) -> None:
        self.id = id
        self.edges: set[str] = set()
        self.junctions: set[str] = set()


class NetworkBuild:
    def __init__(self,
                 dataBase: str,
                 networkFile: str,
                 obsFile: str = None
                 ) -> None:
        self.dataBase = dataBase
        self.networkFile = networkFile
        self.obsFile = obsFile
        self.edges: dict[str, Edge] = {}
        self.lanes: dict[str, NormalLane] = {}
        self.junctions: dict[str, Junction] = {}
        self.junctionLanes: dict[str, JunctionLane] = {}
        self.tlLogics: dict[str, TlLogic] = {}
        # self.obstacles: dict[str, circleObs | rectangleObs] = {}
        self.dataQue = Queue()
        self.geoHashes: dict[tuple[int], geoHash] = {}

    def getEdge(self, eid: str) -> Edge:
        try:
            return self.edges[eid]
        except KeyError:
            return

    def getLane(self, lid: str) -> NormalLane:
        try:
            return self.lanes[lid]
        except KeyError:
            return

    def getJunction(self, jid: str) -> Junction:
        try:
            return self.junctions[jid]
        except KeyError:
            return

    def getJunctionLane(self, jlid: str) -> JunctionLane:
        try:
            return self.junctionLanes[jlid]
        except KeyError:
            return

    def getTlLogic(self, tlid: str) -> TlLogic:
        try:
            return self.tlLogics[tlid]
        except KeyError:
            return

    # def getObstacle(self, obsid: str) -> circleObs | rectangleObs:
    #     try:
    #         return self.obstacles[obsid]
    #     except KeyError:
    #         return

    def affGridIDs(self, centerLine: list[tuple[float]]) -> set[tuple[int]]:
        affGridIDs = set()
        for poi in centerLine:
            poixhash = int(poi[0] // 100)
            poiyhash = int(poi[1] // 100)
            affGridIDs.add((poixhash, poiyhash))

        return affGridIDs

    def insertCommit(self):
        conn = sqlite3.connect(self.dataBase, check_same_thread=False)
        cur = conn.cursor()
        commitCnt = 0
        while not self.dataQue.empty():
            tableName, data, process = self.dataQue.get()
            sql = '{} INTO {} VALUES '.format(process, tableName) + \
                '(' + '?,'*(len(data)-1) + '?' + ')'
            try:
                cur.execute(sql, data)
            except sqlite3.OperationalError as e:
                print(sql, data)
            commitCnt += 1
            if commitCnt == 10000:
                conn.commit()
                commitCnt = 0
        conn.commit()
        cur.close()
        conn.close()

        print('[green bold]Network information commited at {}.[/green bold]'.format(
            datetime.now().strftime('%H:%M:%S.%f')[:-3]))

    def processRawShape(self, rawShape: str) -> list:
        rawList = rawShape.split(' ')
        floatShape = [list(map(float, p.split(','))) for p in rawList]
        return floatShape

    def processEdge(self, eid: str, child: ET.Element):
        if eid[0] == ':':
            for gchild in child:
                if 'disallow' in gchild.attrib:
                    disallow = gchild.attrib['disallow']
                    if disallow == "all":
                        continue
                ilid = gchild.attrib['id']
                try:
                    ilspeed = float(gchild.attrib['speed'])
                except:
                    ilspeed = 13.89
                try:
                    ilwidth = float(gchild.attrib['width'])
                except KeyError:
                    ilwidth = 3.2
                ilLength = float(gchild.attrib['length'])
                self.junctionLanes[ilid] = JunctionLane(
                    id=ilid, width=ilwidth, speed_limit=ilspeed,
                    sumo_length=ilLength,
                )
                self.dataQue.put((
                    'junctionLaneINFO', (
                        ilid, ilwidth, ilspeed, ilLength,
                        None, 0
                    ), 'INSERT'
                ))
        else:
            fromNode = child.attrib['from']
            toNode = child.attrib['to']
            edge = Edge(id=eid, from_junction=fromNode, to_junction=toNode)
            laneNumber = 0
            for gchild in child:
                if gchild.tag == 'lane':
                    # 去除人行道
                    if 'allow' in gchild.attrib: # 有allow属性是人行道
                        # allow = gchild.attrib['allow']
                        continue
                    if 'disallow' in gchild.attrib: # disallow=all是隔离带
                        disallow = gchild.attrib['disallow']
                        if disallow == "all":
                            continue
                    lid = gchild.attrib['id']
                    try:
                        lwidth = float(gchild.attrib['width'])
                    except KeyError:
                        lwidth = 3.2
                    lspeed = float(gchild.attrib['speed'])
                    rawShape = gchild.attrib['shape']
                    lshape = self.processRawShape(rawShape)
                    llength = float(gchild.attrib['length'])
                    lane = NormalLane(id=lid, width=lwidth, speed_limit=lspeed,
                                      sumo_length=llength, affiliated_edge=edge)
                    self.dataQue.put((
                        'laneINFO', (
                            lid, rawShape, lwidth, lspeed, eid, llength
                        ), 'INSERT'
                    ))
                    shapeUnzip = list(zip(*lshape))

                    # interpolate shape points for better represent shape
                    shapeUnzip = [
                        np.interp(
                            np.linspace(0, len(shapeUnzip[0])-1, 50),
                            np.arange(0, len(shapeUnzip[0])),
                            shapeUnzip[i]
                        ) for i in range(2)
                    ]
                    lane.course_spline = Spline2D(shapeUnzip[0], shapeUnzip[1])
                    lane.getPlotElem()
                    self.lanes[lid] = lane
                    edge.lanes.add(lane.id)
                    laneAffGridIDs = self.affGridIDs(lane.center_line)
                    edge.affGridIDs = edge.affGridIDs | laneAffGridIDs
                    laneNumber += 1
            edge.lane_num = laneNumber
            for gridID in edge.affGridIDs:
                try:
                    geohash = self.geoHashes[gridID]
                except KeyError:
                    geohash = geoHash(gridID)
                    self.geoHashes[gridID] = geohash
                geohash.edges.add(eid)
            self.edges[eid] = edge
            self.dataQue.put((
                'edgeINFO', (eid, laneNumber, fromNode, toNode), 'INSERT'
            ))

    def processConnection(self, child: ET.Element):
        fromEdgeID = child.attrib['from']
        fromEdge = self.getEdge(fromEdgeID)
        fromLaneIdx = child.attrib['fromLane']
        fromLaneID = fromEdgeID + '_' + fromLaneIdx
        fromLane = self.getLane(fromLaneID)
        toEdgeID = child.attrib['to']
        toLaneIdx = child.attrib['toLane']
        toLaneID = toEdgeID + '_' + toLaneIdx
        toLane = self.getLane(toLaneID)
        if fromLane and toLane:
            direction = child.attrib['dir']
            junctionLaneID = child.attrib['via']
            junctionLane = self.getJunctionLane(junctionLaneID)
            self.dataQue.put((
                'connectionINFO', (
                    fromLaneID, toLaneID, direction, junctionLaneID
                ), 'INSERT'
            ))
            if junctionLane.sumo_length < 1:
                fromLane.next_lanes[toLaneID] = (toLaneID, 's')
                fromEdge.next_edge_info[toEdgeID].add(fromLaneID)
            else:
                # junctionLane = self.getJunctionLane(junctionLaneID)
                if 'tl' in child.attrib.keys():
                    tl = child.attrib['tl']
                    linkIndex = int(child.attrib['linkIndex'])
                    junctionLane.tlLogic = tl
                    junctionLane.tlsIndex = linkIndex
                self.dataQue.put((
                    'junctionLaneINFO', (
                        junctionLane.id, junctionLane.width,
                        junctionLane.speed_limit,
                        junctionLane.sumo_length,
                        junctionLane.tlLogic,
                        junctionLane.tlsIndex
                    ), 'REPLACE'
                ))
                center_line = []
                for si in np.linspace(
                    fromLane.course_spline.s[-1] - OVERLAP_DISTANCE,
                    fromLane.course_spline.s[-1], num=20
                ):
                    center_line.append(
                        fromLane.course_spline.calc_position(si))
                for si in np.linspace(0, OVERLAP_DISTANCE, num=20):
                    center_line.append(
                        toLane.course_spline.calc_position(si)
                    )
                junctionLane.course_spline = Spline2D(
                    list(zip(*center_line))[0], list(zip(*center_line))[1]
                )
                junctionLane.getPlotElem()
                junctionLane.last_lane_id = fromLaneID
                junctionLane.next_lane_id = toLaneID
                fromLane.next_lanes[toLaneID] = (junctionLaneID, direction)
                fromEdge.next_edge_info[toEdgeID].add(fromLaneID)
                # add this junctionLane to it's parent Junction's JunctionLanes
                fromEdge = self.getEdge(fromEdgeID)
                juncID = fromEdge.to_junction
                junction = self.getJunction(juncID)
                junctionLane.affJunc = juncID
                jlAffGridIDs = self.affGridIDs(junctionLane.center_line)
                junction.affGridIDs = junction.affGridIDs | jlAffGridIDs
                junction.JunctionLanes.add(junctionLaneID)

    def getData(self):
        elementTree = ET.parse(self.networkFile)
        root = elementTree.getroot()
        for child in root:
            if child.tag == 'edge':
                eid = child.attrib['id']
                # Some useless internal lanes will be generated by the follow codes.
                self.processEdge(eid, child)
            elif child.tag == 'junction':
                jid = child.attrib['id']
                junc = Junction(jid)
                if jid[0] != ':':
                    intLanes = child.attrib['intLanes']
                    if intLanes:
                        intLanes = intLanes.split(' ')
                        for il in intLanes:
                            ilins = self.getJunctionLane(il)
                            if ilins:
                                ilins.affJunc = jid
                                junc.JunctionLanes.add(il)
                    jrawShape = child.attrib['shape']
                    juncShape = self.processRawShape(jrawShape)
                    # Add the first point to form a closed shape
                    juncShape.append(juncShape[0])
                    junc.shape = juncShape
                    self.junctions[jid] = junc
                    self.dataQue.put((
                        'junctionINFO', (jid, jrawShape), 'INSERT'
                    ))
            elif child.tag == 'connection':
                # in .net.xml, the elements 'edge' come first than elements
                # 'connection', so the follow codes can work well.
                self.processConnection(child)
            elif child.tag == 'tlLogic':
                tlid = child.attrib['id']
                tlType = child.attrib['type']
                preDefPhases = []
                for gchild in child:
                    if gchild.tag == 'phase':
                        preDefPhases.append(gchild.attrib['state'])

                self.tlLogics[tlid] = TlLogic(tlid, tlType, preDefPhases)
                self.dataQue.put((
                    'tlLogicINFO',
                    (tlid, tlType, ' '.join(preDefPhases)), 'INSERT'
                ))
        for junction in self.junctions.values():
            for gridID in junction.affGridIDs:
                try:
                    geohash = self.geoHashes[gridID]
                except KeyError:
                    geohash = geoHash(gridID)
                    self.geoHashes[gridID] = geohash
                geohash.junctions.add(junction.id)

        for ghid, ghins in self.geoHashes.items():
            ghx, ghy = ghid
            ghEdges = ','.join(ghins.edges)
            ghJunctions = ','.join(ghins.junctions)
            self.dataQue.put((
                'geohashINFO',
                (ghx, ghy, ghEdges, ghJunctions), 'INSERT'
            ))

    def buildTopology(self):
        for eid, einfo in self.edges.items():
            fj = self.getJunction(einfo.from_junction)
            tj = self.getJunction(einfo.to_junction)
            fj.outgoing_edges.add(eid)
            tj.incoming_edges.add(eid)

        print('[green bold]Network building finished at {}.[/green bold]'.format(
            datetime.now().strftime('%H:%M:%S.%f')[:-3]))

        Th = Thread(target=self.insertCommit)
        Th.start()
    
    def plotLane(
            self, lane: NormalLane, flag: int, ex: float, ey: float,
            widget: QWidget, ctf: CoordTF, is_selectable: bool
    ):
        lid = lane.id        
        left_bound_tf = [
            ctf.qtCoord(wp[0], wp[1], ex, ey) for wp in lane.left_bound
        ]
        right_bound_tf = [
            ctf.qtCoord(wp[0], wp[1], ex, ey) for wp in lane.right_bound
        ]
        widget.drawPolyline(left_bound_tf, QColor(255,255,255), 1)
        # graphics_items.append(graphicsItem('polyline',left_bound_tf, QColor(255,255,255),QColor(0,0,0), 1))
        if flag:
            widget.drawPolyline(right_bound_tf, QColor(255,255,255), 1)
            # graphics_items.append(graphicsItem('polyline',right_bound_tf, QColor(255,255,255), QColor(0,0,0), 1))

        left_bound_tf.reverse()
        all_bound_tf = right_bound_tf + left_bound_tf
        all_bound_tf.append(right_bound_tf[0])
        widget.drawPolygon(all_bound_tf, QColor(255, 255, 255), QColor(255, 255, 255, 30), 0, is_selectable, lid)
        # graphics_items.append(graphicsItem('polygon',all_bound_tf, QColor(255, 255, 255),QColor(255, 255, 255, 30), 0, is_selectable, lid))

    
    def plotEdge(self, eid: str, widget: QWidget, ex: float, ey: float,
                 ctf: CoordTF, is_selectable: bool = False):
        '''
        edge: 某个方向几条车道的集合
        '''
        edge = self.getEdge(eid)
        lane_index = 0
        lanes = list(edge.lanes)
        lanes.sort()        
        for lane_id in lanes:
            lane = self.getLane(lane_id)
            flag = lane_index == 0
            self.plotLane(lane, flag, ex, ey, widget, ctf, is_selectable)
            lane_index += 1

        # 根据左右边界获取 edge 的封闭图形
        # left_bound_tf.reverse()
        # x = right_bound_tf[15][0]
        # y = right_bound_tf[15][1]
        # right_bound_tf.extend(left_bound_tf)
        # right_bound_tf.append(right_bound_tf[0])
        # widget.drawPolygon(right_bound_tf, pen_color=QColor(0, 0, 0), pen_width=2, brush_color=QColor(0,0,0,30), is_selectable=True, data=eid)

        # widget.addText(x, y, eid)

    def plotJunctionLane(self, jlid: str, widget: QWidget, ex: float, ey: float,
                         ctf: CoordTF):
        juncLane = self.getJunctionLane(jlid)
        if juncLane:
            try:
                center_line = juncLane.center_line
            except AttributeError:
                return
            # center_line_tf = [
            #     ctf.qtCoord(wp[0], wp[1], ex, ey) for wp in center_line
            # ]
            if juncLane.currTlState:
                if juncLane.currTlState == 'r':
                    # jlColor = (232, 65, 24)
                    jlColor = QColor(255, 107, 129, 100)
                elif juncLane.currTlState == 'y':
                    jlColor = QColor(251, 197, 49, 100)
                elif juncLane.currTlState == 'g' or juncLane.currTlState == 'G':
                    jlColor = QColor(39, 174, 96, 100)
            else:
                jlColor = QColor(255, 255, 255, 50)
            right_bound_tf = [
                ctf.qtCoord(wp[0], wp[1], ex, ey) for wp in juncLane.right_bound
            ]
            left_bound_tf = [
                ctf.qtCoord(wp[0], wp[1], ex, ey) for wp in juncLane.left_bound
            ]
            left_bound_tf.reverse()
            right_bound_tf.extend(left_bound_tf)
            right_bound_tf.append(right_bound_tf[0])
            widget.drawPolygon(right_bound_tf, jlColor, jlColor, 1)
            # graphics_items.append(graphicsItem('polygon', right_bound_tf, jlColor, jlColor, 1))
            # signal.emit('polygon', right_bound_tf, jlColor, jlColor, 1, True, jlid)
            # signal.emit('polyline', center_line_tf, jlColor, jlColor, 12, False, "")
            # if flag:
            #     right_bound_tf = [
            #         ctf.qtCoord(wp[0], wp[1], ex, ey) for wp in juncLane.right_bound
            #     ]
            #     signal.emit('polyline', right_bound_tf, QColor(100, 149, 237), QColor(0, 0, 0), 2, False, "")
            # widget.drawPolyline(center_line_tf, color=jlColor,width=15)

    
    def plotJunction(
            self, jid: str, widget: QWidget, ex: float, ey: float, ctf: CoordTF):
        junction = self.getJunction(jid)
        polyShape = [ctf.qtCoord(p[0], p[1], ex, ey) for p in junction.shape]
        widget.drawPolyline(polyShape, QColor(255, 255, 255, 70), 2)
        # graphics_items.append(graphicsItem('polyline', polyShape, QColor(255, 255, 255, 70), QColor(0, 0, 0), 2))

        for jl in junction.JunctionLanes:
            self.plotJunctionLane(jl, widget, ex, ey, ctf)
    
    def plotScene(self, widget: QWidget, ex:float, ey:float, ctf: CoordTF, is_selectable: bool = False):
        if self.edges:
            for ed in self.edges:
                self.plotEdge(ed, widget, ex, ey, ctf, is_selectable)

        if self.junctions:
            for jc in self.junctions:
                self.plotJunction(jc, widget, ex, ey, ctf)
    
    def showTextOnEdge(self, signal: Signal, eid: str, text: str, ex:float, ey:float, ctf: CoordTF):
        edge = self.getEdge(eid)
        lane_index = 0
        lanes = list(edge.lanes)
        lanes.sort()        
        lane_num = len(lanes)
        for lane_id in lanes:
            lane = self.getLane(lane_id)
            flag = 0b00
            if lane_index == 0:
                flag += 1
            if lane_index == lane_num - 1:
                flag += 2
            if flag & 0b10:
                left_bound_tf = [
                    ctf.qtCoord(wp[0], wp[1], ex, ey) for wp in lane.left_bound
                ]
            if flag & 0b01:
                right_bound_tf = [
                    ctf.qtCoord(wp[0], wp[1], ex, ey) for wp in lane.right_bound
                ]
            lane_index += 1

        index = 25
        x = min(left_bound_tf[index][0], right_bound_tf[index][0])
        y = min(left_bound_tf[index][1], right_bound_tf[index][1])
        # widget.addText(x, y, text, fontsize=15)


class Rebuild(NetworkBuild):
    def __init__(self,
                 dataBase: str,
                 ) -> None:
        networkFile: str = None,
        obsFile: str = None
        super().__init__(dataBase, networkFile, obsFile)

    def getData(self):
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()

        cur.execute('SELECT * FROM junctionINFO;')
        junctionINFO = cur.fetchall()
        if junctionINFO:
            for ji in junctionINFO:
                junctionID = ji[0]
                jrawShape = ji[1]
                juncShape = self.processRawShape(jrawShape)
                # Add the first point to form a closed shape
                juncShape.append(juncShape[0])
                junc = Junction(junctionID)
                junc.shape = juncShape
                self.junctions[junctionID] = junc

        cur.execute('SELECT * FROM edgeINFO;')
        edgeInfo = cur.fetchall()
        if edgeInfo:
            for ed in edgeInfo:
                eid, laneNumber, fromJunction, toJunction = ed
                self.edges[eid] = Edge(
                    id=eid, lane_num=laneNumber,
                    from_junction=fromJunction,
                    to_junction=toJunction
                )

        cur.execute('SELECT * FROM laneINFO;')
        laneINFO = cur.fetchall()
        if laneINFO:
            for la in laneINFO:
                lid, rawShape, lwidth, lspeed, eid, llength = la
                lshape = self.processRawShape(rawShape)
                lane = NormalLane(lid, lwidth, lspeed, eid, llength)
                lane = NormalLane(
                    id=lid, width=lwidth, speed_limit=lspeed,
                    affiliated_edge=self.getEdge(eid), sumo_length=llength
                )
                shapeUnzip = list(zip(*lshape))
                # interpolate shape points for better represent shape
                shapeUnzip = [
                    np.interp(
                        np.linspace(0, len(shapeUnzip[0])-1, 50),
                        np.arange(0, len(shapeUnzip[0])),
                        shapeUnzip[i]
                    ) for i in range(2)
                ]
                lane.course_spline = Spline2D(shapeUnzip[0], shapeUnzip[1])
                lane.getPlotElem()
                self.lanes[lid] = lane
                self.getEdge(eid).lanes.add(lid)

        cur.execute('SELECT * FROM junctionLaneINFO;')
        JunctionLaneINFO = cur.fetchall()
        if JunctionLaneINFO:
            for jl in JunctionLaneINFO:
                jlid, jlwidth, jlspeed, jlLength, tlLogicID, tlsIndex = jl
                self.junctionLanes[jlid] = JunctionLane(
                    id=jlid, width=jlwidth,
                    speed_limit=jlspeed, sumo_length=jlLength,
                    tlLogic=tlLogicID, tlsIndex=tlsIndex
                )

        cur.execute('SELECT * FROM tlLogicINFO;')
        tlLogicINFO = cur.fetchall()
        if tlLogicINFO:
            for tll in tlLogicINFO:
                tlid, tlType, preDefPhases = tll
                self.tlLogics[tlid] = TlLogic(
                    tlid, tlType, preDefPhases.split(' '))

        cur.execute('SELECT * FROM connectionINFO;')
        connectionINFO = cur.fetchall()
        if connectionINFO:
            for ci in connectionINFO:
                fromLaneID, toLaneID, direction, junctionLaneID = ci
                fromLane = self.getLane(fromLaneID)
                fromEdgeID = deduceEdge(fromLaneID)
                fromEdge = self.getEdge(fromEdgeID)
                junctionLane = self.getJunctionLane(junctionLaneID)
                if not junctionLane:
                    print(
                        'The JunctionLane is not found in database: ',
                        junctionLaneID
                    )
                toEdgeID = deduceEdge(toLaneID)
                if junctionLane.sumo_length < 1:
                    fromLane.next_lanes[toLaneID] = (toLaneID, 's')
                    fromEdge.next_edge_info[toEdgeID].add(fromLaneID)
                else:
                    junctionLane = self.getJunctionLane(junctionLaneID)
                    fromEdgeID = deduceEdge(fromLaneID)
                    center_line = []
                    for si in np.linspace(
                        fromLane.course_spline.s[-1] - OVERLAP_DISTANCE,
                        fromLane.course_spline.s[-1], num=20
                    ):
                        center_line.append(
                            fromLane.course_spline.calc_position(si))
                    for si in np.linspace(0, OVERLAP_DISTANCE, num=20):
                        center_line.append(
                            self.getLane(
                                toLaneID).course_spline.calc_position(si)
                        )
                    junctionLane.course_spline = Spline2D(
                        list(zip(*center_line))[0], list(zip(*center_line))[1]
                    )
                    junctionLane.getPlotElem()
                    junctionLane.last_lane_id = fromLaneID
                    junctionLane.next_lane_id = toLaneID
                    fromLane.next_lanes[toLaneID] = (
                        junctionLaneID, direction)
                    fromEdge.next_edge_info[toEdgeID].add(fromLaneID)
                    # add this junctionLane to it's parent Junction's JunctionLanes
                    fromEdge = self.getEdge(fromEdgeID)
                    junction = self.getJunction(fromEdge.to_junction)
                    junction.JunctionLanes.add(junctionLaneID)

        cur.execute('SELECT * FROM geohashINFO;')
        geohashINFO = cur.fetchall()
        if geohashINFO:
            for gi in geohashINFO:
                ghx, ghy, ghEdges, ghJunctions = gi
                ghID = (ghx, ghy)
                geohash = geoHash(ghID)
                if ghEdges:
                    geohash.edges = set(ghEdges.split(','))
                if ghJunctions:
                    geohash.junctions = set(ghJunctions.split(','))
                self.geoHashes[ghID] = geohash

        cur.close()
        conn.close()

    def buildTopology(self):
        for k, v in self.edges.items():
            fj = self.getJunction(v.from_junction)
            tj = self.getJunction(v.to_junction)
            fj.outgoing_edges.add(k)
            tj.incoming_edges.add(k)

        print('[green bold]Network building finished at {}.[/green bold]'.format(
            datetime.now().strftime('%H:%M:%S.%f')[:-3]))