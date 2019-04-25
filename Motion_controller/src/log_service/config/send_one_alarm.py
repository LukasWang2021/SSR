#! /usr/bin/env python2
# -*- coding: utf-8 -*-
import json
import datetime
import tornado.web
import tornado.httpclient


def send_one_alarm(event_record):
    """����post������"""

    one_record = dict()
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    one_record['time'] = current_time
    one_record['code'] = event_record[0:16]
    one_record['detailEn'] = [event_record[16:]]
    one_record['detailCn'] = [event_record[16:]]
    req = tornado.httpclient.HTTPRequest(
        url='http://0.0.0.0:9003/',
        method='POST',
        body=json.dumps({'cmd': 'alarm/append_one_alarm', 'param': one_record}),
        connect_timeout=5,
        validate_cert=False)
    status_code = 1001
    try:
        http_client = tornado.httpclient.HTTPClient()
        rsp = http_client.fetch(req)
        server_response = json.loads(rsp.body)
        http_client.close()
        if server_response is None:
            raise Exception('files server not process the request')
        else:
            execute_result = server_response['param']
            status_code = server_response['statusCode']
            if server_response['statusCode'] != 1000:
                #print('send_one_alarm error:', json.dumps(execute_result))

    except tornado.httpclient.HTTPError as e:
        if '599' in str(e):
            err_info = 'ECONNREFUSED'
            status_code = 1599
        elif '598' in str(e):
            err_info = 'READTIMEOUT'
            status_code = 1598
        else:
            err_info = str(e)
        print('send_one_alarm HTTPError:', json.dumps({'result': err_info}))
    except Exception as e:
        print('send_one_alarm Exception', str(e))
    return status_code