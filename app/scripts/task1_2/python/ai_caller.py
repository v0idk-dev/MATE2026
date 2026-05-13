#!/usr/bin/env python3
# Unified AI caller. Posts a structured prompt to the chosen provider via
# the Electron main process's :5002 /ai_call proxy (which adds the keychain
# key and forwards to the real provider). Returns parsed JSON or raises.
import base64, json, os, sys, urllib.request

PROXY = 'http://127.0.0.1:5002/ai_call'

PROMPT_HYBRID = (
  "You are assisting a stereo photogrammetry pipeline. You are given LEFT "
  "and RIGHT rectified images of a PVC coral garden with colored 10x10cm "
  "plates. Identify each plate's CENTER pixel coordinates in BOTH images, "
  "and the four CORNER coordinates of each plate in BOTH images. Return "
  "STRICT JSON with this schema and nothing else:\n"
  '{"plates":[{"id":int,"color":"string",'
  '"center_left":{"x":float,"y":float},"center_right":{"x":float,"y":float},'
  '"corners_left":[{"x":float,"y":float}x4],'
  '"corners_right":[{"x":float,"y":float}x4]}],'
  '"warnings":["..."]}'
)
PROMPT_AI_ONLY = (
  "You are looking at images of a PVC coral garden built from 1/2 inch PVC "
  "pipe with EIGHT 10 cm x 10 cm colored squares attached as targets. The "
  "garden is between 1m and 2.5m long, ~36cm wide, with some height. The "
  "image may include a ruler for scale.\n\n"
  "Identify ALL EIGHT colored target squares and return their 3D position "
  "and four corner coordinates in METERS in the left-camera frame. Also "
  "identify all visible pipe segments with endpoints in 3D and per-pipe "
  "length. Estimate overall LENGTH (longest dimension) and HEIGHT (vertical "
  "extent) of the structure in METERS.\n\n"
  "Return STRICT JSON, no markdown, no commentary:\n"
  '{"plates":[{"id":int,"color":"string",'
  '"position_3d":{"x":float,"y":float,"z":float},'
  '"corners_3d":[{"x":float,"y":float,"z":float},{"x":float,"y":float,"z":float},{"x":float,"y":float,"z":float},{"x":float,"y":float,"z":float}]}],'
  '"pipes":[{"a":{"x":float,"y":float,"z":float},"b":{"x":float,"y":float,"z":float},"length":float}],'
  '"length":float,"height":float,'
  '"warnings":["string"]}\n'
  "Use METERS throughout. Aim for all 8 plates."
)

def _b64(path):
    with open(path, 'rb') as f: return base64.b64encode(f.read()).decode()

def _build_messages(provider, prompt, images_b64):
    # OpenAI/Anthropic: chat/messages; Google: contents.
    if provider == 'openai':
        content = [{'type': 'text', 'text': prompt}]
        for b in images_b64:
            content.append({'type':'image_url',
                'image_url':{'url':f'data:image/jpeg;base64,{b}'}})
        return {'model': None,  # filled by caller
                'messages':[{'role':'user','content':content}],
                'max_tokens': 4096}
    if provider == 'anthropic':
        content = []
        for b in images_b64:
            content.append({'type':'image','source':{'type':'base64',
                'media_type':'image/jpeg','data':b}})
        content.append({'type':'text','text':prompt})
        return {'model': None, 'max_tokens': 4096,
                'messages':[{'role':'user','content':content}]}
    if provider == 'google':
        parts = [{'text': prompt}]
        for b in images_b64:
            parts.append({'inline_data':{'mime_type':'image/jpeg','data':b}})
        return {'contents':[{'role':'user','parts':parts}]}
    if provider == 'apple':
        # Apple on-device cannot consume images via this proxy yet.
        return {'prompt': prompt}
    raise ValueError(f'unknown provider {provider}')

def _extract_text(provider, resp_body):
    j = json.loads(resp_body)
    if provider == 'openai':
        return j['choices'][0]['message']['content']
    if provider == 'anthropic':
        return ''.join(b.get('text','') for b in j.get('content',[]))
    if provider == 'google':
        return j['candidates'][0]['content']['parts'][0]['text']
    if provider == 'apple':
        return j.get('text','')
    return ''

def _extract_json_block(text):
    s = text.find('{'); e = text.rfind('}')
    if s < 0 or e < s: raise ValueError('no JSON object in model output')
    return json.loads(text[s:e+1])

def call(provider, model, prompt_kind, image_paths):
    images_b64 = [_b64(p) for p in image_paths]
    prompt = PROMPT_HYBRID if prompt_kind == 'hybrid' else PROMPT_AI_ONLY
    body = _build_messages(provider, prompt, images_b64)
    body['model'] = model  # noop for google/apple, set for openai/anthropic
    if provider == 'google': body.pop('model', None)
    payload = json.dumps({'provider': provider, 'model': model, 'body': body}).encode()
    req = urllib.request.Request(PROXY, data=payload,
        headers={'Content-Type':'application/json'}, method='POST')
    with urllib.request.urlopen(req, timeout=180) as r:
        text = _extract_text(provider, r.read().decode('utf-8'))
    return _extract_json_block(text)

if __name__ == '__main__':
    # CLI: ai_caller.py <provider> <model> <kind> <img1> [img2...]
    args = sys.argv[1:]
    out = call(args[0], args[1], args[2], args[3:])
    print(json.dumps(out))
