#!/usr/bin/env python3
"""Read-only dataset / PI05 processor and projection-head smoke test.

Run with /data/lerobot/.venv/bin/python. No robot connection, dataset edits,
downloads, pretrained weights, RTC, or full PaliGemma allocation. The lightweight
vector field exercises PI05's real projection modules and flow primitives;
it deliberately does NOT represent PI05Policy.forward or a learned controller.
"""
import argparse
import copy
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def inventory(root):
    return {str(p.relative_to(root)):sha256(p) for p in sorted(root.rglob('*')) if p.is_file()}


def tensor_info(value):
    return dict(shape=list(value.shape), dtype=str(value.dtype),
                minimum=float(value.min()), maximum=float(value.max()))


def write_readable_report(output, report):
    def table(headers, rows):
        return '\n'.join(['| '+' | '.join(headers)+' |',
                          '| '+' | '.join(['---']*len(headers))+' |']+
                         ['| '+' | '.join(map(str,row))+' |' for row in rows])
    def fmt(info):
        return f"{info['shape']}, {info['dtype']}, [{info['minimum']:.6g}, {info['maximum']:.6g}]"
    batches=report['batches']
    text=['# 30Hz 三相机数据集与 PI0.5 接口冒烟测试',
        f"数据集：`{report['dataset']}`。Python：`/data/lerobot/.venv/bin/python`。",
        '**第一步完整通过；第二步采用归一化与真实Action Expert投影头的轻量接口测试，通过loss、反向梯度和7维反归一化。未运行完整PI05Policy.forward，未加载PaliGemma/Action Expert Transformer或预训练权重。**',
        f"原始数据集全部{len(report['dataset_sha256_before'])}个文件测试前后SHA256完全一致：{report['dataset_unchanged']}。HF数据缓存与图像输出均在独立审计目录内。",
        '## Action Chunk与末尾padding',
        '实际使用LeRobotDataset(delta_timestamps={action: [i/30.0 for i in range(50)]})，DataLoader(batch_size=2, num_workers=0)。逐个比较100个动作的索引、值、padding标记，末尾重复最后一个动作。50个动作位置是0至49/30秒，chunk执行时长通常按50/30=1.667秒理解。',
        table(['batch','数据集帧号','state','action','action_is_pad','每条padding步数'],
              [[name,b['indices'],b['raw']['observation.state']['shape'],b['raw']['action']['shape'],
                b['raw']['action_is_pad']['shape'],b['padding_counts']] for name,b in batches.items()]),
        '夹剪batch的首个动作正好是0→1的闭合边界，且两帧均标为clamp_cut；尾部batch取1045/1046，两帧均为release。task均正确加载为：采摘下画面里面所有的成熟番茄串并放入收集筐。',
        table(['batch / camera','原始shape / dtype / range','PI05预处理shape / dtype / range'],
              [[name+' / '+key.removeprefix('observation.images.'),fmt(b['raw'][key]),fmt(v)]
               for name,b in batches.items() for key,v in b['pi05_images'].items()]),
        '## 分位数与归一化',
        'dataset.meta.stats原本已有q01和q99，不需要修改导出器或数据集。七个维度的跨度全部有限且严格大于零：',
        table(['特征','q99−q01（逐维）','最小跨度'],
              [[key,', '.join(f'{v:.6f}' for v in q['spans']),f"{q['minimum_span']:.6f}"]
               for key,q in report['quantiles'].items()]),
        '使用实际NormalizerProcessorStep/UnnormalizerProcessorStep及PI05Config默认QUANTILES规则。两个batch动作归一化→反归一化最大绝对误差分别为'+
        ', '.join(f"{b['normalization_roundtrip_max_abs_error']:.3g}" for b in batches.values())+'。图像为IDENTITY；实际PI05图像预处理内部再映射到[-1,1]。分位数归一化并不保证所有样本都落入[-1,1]，这本身不属于NaN/Inf问题。',
        '## 224×224视觉预览',
        '直接调用本地PI05Policy._preprocess_images，使用真实resize_with_pad_torch等比例缩放、居中黑边及[-1,1]变换。L515有效画面约224×126，D405约224×168；没有拉伸成正方形。',
        '![三相机原图与224×224输入对比](visual_comparison.png)',
        '预览每个相机左侧是为排版缩小的原始画面，右侧是原生224像素模型输入。完整分辨率原图与224图另存为clamp_cut_*和episode_tail_*.png。当前临时右L515以墙面为主，腕相机在闭合时未清楚展示剪口；本测试确认的是视觉张量处理，不能替代安装后的剪口可见性验收。',
        '## PI05配置映射与资源',
        table(['设置','实际值'],[
            ['外部特征','3路RGB + observation.state(7) + action(7)'],
            ['chunk_size / n_action_steps','50 / 50'],['内部max_state_dim / max_action_dim','32 / 32'],
            ['image_resolution','224×224'],['relative actions','False，保留绝对关节/工具目标'],
            ['RTC / memory / compile','禁用'],['Action Expert投影宽度',report['projection_width']],
            ['实例化投影参数数',report['projection_parameters']]]),
        '资源探测：'+json.dumps(report['resources'],ensure_ascii=False)+'。',
        '默认配置是float32，3B参数仅权重下界约12GB，尚未包含视觉路径、激活或梯度；当前8GB GPU与可用系统内存不足以稳妥运行完整训练前向。环境还缺transformers，没有缓存PaliGemma tokenizer/预训练权重。本次使用用户允许的投影头路径，不下载/安装模型依赖。',
        '实例化本地PI05Pytorch使用的32→1024 action_in_proj、时间MLP、1024→32 action_out_proj；调用真实embed_suffix、prepare_action、sample_noise/sample_time、_build_flow_matching_inputs。诊断向量场明确为action_out_proj(action_in_proj(x_t)+time_condition)，省略Transformer；三路图像和状态提示词处理分别测试，但不作为这个投影向量场的条件输入。',
        '## Flow matching与推理解码',
        table(['batch','F.mse_loss实际维度均值','排除padding的诊断均值','梯度有限'],
              [[name,f"{b['flow_matching']['loss']:.8f}",f"{b['flow_matching']['valid_only_loss']:.8f}",
                b['flow_matching']['all_gradients_finite']] for name,b in batches.items()]),
        '采用x_t=(1−t)action+t·noise，u_t=noise−action；u_t/v_t均为[2,50,32]，损失按真实动作维度截取前7维。两个batch的loss、投影头反向梯度均为有限数。',
        '**本地PI05Policy.forward当前并未把action_is_pad用于loss reduction**，尾部padding会参与原生损失均值。本报告额外给出valid_only_loss作诊断；没有改写模型或损失函数，也不宣称训练时已正确屏蔽padding。',
        '调用同一euler_integrate原语做10步随机投影向量场采样，裁去32维padding后得到[2,50,7]，使用真实Unnormalizer恢复前六维rad、第七维0=open/1=closed的绝对工具尺度。结果全部有限并保存projection_decoded_actions.npy。',
        '示例首个动作：'+json.dumps(report['inference']['first_action'])+'。',
        '随机未训练输出可能超出训练分位数区间，第七维也可能超出[0,1]；它只是连续物理尺度的接口解码，不是已经完成二值开闭决策或可执行机器人指令。没有发送任何运动或工具命令。',
        '## 测试边界与产物',
        *['- '+v for v in report['limitations']],
        'smoke_report.json保存全部实际shape、dtype、range、q01/q99、两份batch的状态提示词、loss、资源、源代码哈希及数据集前后SHA256。stdout.log保留终端打印。脚本可独立复现：',
        '```bash\n/data/lerobot/.venv/bin/python /home/lcw/VR_teleoperation/scripts/smoke_aubo_pi05.py \\\n  --dataset '+report['dataset']+' \\\n  --output /data/VR_teleoperation_runtime/audits/pi05-smoke-new-run\n```']
    (output/'REPORT.md').write_text('\n\n'.join(text)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root, output = args.dataset.resolve(), args.output.resolve()
    if root == output or root in output.parents:
        raise ValueError('Output must be outside the original dataset')
    output.mkdir(parents=True, exist_ok=False)
    os.environ['HF_HUB_OFFLINE'] = '1'
    os.environ['HF_DATASETS_CACHE'] = str(output/'hf_cache')
    before = inventory(root)
    report = dict(complete=False, dataset=str(root), output=str(output), checks={}, limitations=[])
    started = time.monotonic()
    try:
        import numpy as np
        import torch
        from torch import nn
        import torch.nn.functional as F
        from torch.utils.data import DataLoader, Subset
        from PIL import Image, ImageDraw, ImageFont
        from lerobot.configs import FeatureType, PolicyFeature
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.lerobot_types import TransitionKey as TK
        from lerobot.processor.normalize_processor import NormalizerProcessorStep, UnnormalizerProcessorStep
        from lerobot.policies.pi05.configuration_pi05 import PI05Config
        from lerobot.policies.pi05.processor_pi05 import Pi05PrepareStateTokenizerProcessorStep
        from lerobot.policies.pi05 import modeling_pi05 as modeling
        from lerobot.policies.common.flow_matching import euler_integrate

        torch.manual_seed(20261115)
        torch.set_num_threads(2)
        mem = {line.split(':')[0]:int(line.split()[1])*1024
               for line in Path('/proc/meminfo').read_text().splitlines()}
        cuda = torch.cuda.is_available()
        resources = dict(system_total_bytes=mem['MemTotal'], system_available_bytes=mem['MemAvailable'],
            cuda_available=cuda, transformers_available=importlib.util.find_spec('transformers') is not None,
            full_model_fp32_weights_lower_bound_bytes=3_000_000_000*4,
            full_model_training_activations_and_gradients_additional=True)
        if cuda:
            free, total = torch.cuda.mem_get_info()
            resources.update(gpu_name=torch.cuda.get_device_name(), gpu_free_bytes=free, gpu_total_bytes=total)
        report['resources'] = resources
        report['versions'] = {name:importlib.metadata.version(name) for name in ('torch', 'lerobot', 'numpy')}
        report['model_source'] = str(Path(modeling.__file__).resolve())
        report['model_source_sha256'] = sha256(Path(modeling.__file__))
        report['execution_mode'] = 'real_PI05_processors_and_projection_heads_only'
        report['full_policy_forward_tested'] = False
        report['limitations'] += [
            '8GB laptop GPU and available RAM do not provide a safe budget for the default float32 3B backbone, activations and gradients; full allocation skipped.',
            'transformers/tokenizer/pretrained checkpoint not loaded; no downloads and no complete language/vision transformer forward.',
            'Projection-only diagnostic vector field omits Action Expert transformer and visual/language conditioning. Finite loss is a tensor-interface check, not model quality.',
            'Random untrained inference is never sent to a robot; dim7 remains a continuous normalized tool value until a deployment-specific binary decoder is applied.',
        ]
        print('RESOURCES', json.dumps(resources), flush=True)
        export = json.loads((root/'export_report.json').read_text())
        delta = {'action':[i/30.0 for i in range(50)]}
        dataset = LeRobotDataset(export['repo_id'],root=root,delta_timestamps=delta,
                                 download_videos=False,video_backend='pyav')
        assert dataset.fps == 30 and dataset.num_frames == 1047
        precise = dataset.hf_dataset.with_format(None)
        scalar = lambda row,key:row[key][0] if isinstance(row[key],list) else row[key]
        table = [precise[i] for i in range(dataset.num_frames)]
        targets = np.asarray([row['action'] for row in table])
        stage_index = export['enums']['task_stage'].index('clamp_cut')
        closes = [i for i in range(1,len(table)) if targets[i,6] == 1 and targets[i-1,6] == 0
                  and int(scalar(table[i],'source.task_stage')) == stage_index]
        assert closes, 'No actual close transition in clamp_cut'
        close = closes[0]
        selections = {'clamp_cut':[close,close+1], 'episode_tail':[len(table)-2,len(table)-1]}
        cameras = ['observation.images.'+key for key in export['active_cameras']]
        features = {'observation.state':PolicyFeature(type=FeatureType.STATE,shape=(7,)),
                    'action':PolicyFeature(type=FeatureType.ACTION,shape=(7,))}
        for key in cameras:
            h,w,c = dataset.features[key]['shape']
            features[key] = PolicyFeature(type=FeatureType.VISUAL,shape=(c,h,w))
        config = PI05Config(input_features={k:v for k,v in features.items() if k!='action'},
                            output_features={'action':features['action']}, device='cpu',
                            chunk_size=50,n_action_steps=50,image_resolution=(224,224),
                            use_relative_actions=False,rtc_config=None,compile_model=False)
        config.validate_features()
        assert config.action_delta_indices == list(range(50))
        assert config.max_state_dim == config.max_action_dim == 32
        report['config'] = json.loads(config.to_json()) if hasattr(config,'to_json') else dict(
            chunk_size=50,max_state_dim=32,max_action_dim=32,image_resolution=[224,224],
            input_features={k:dict(shape=list(v.shape),type=v.type.value) for k,v in config.input_features.items()},
            output_features={'action':dict(shape=[7],type='ACTION')},use_relative_actions=False,
            normalization_mapping={k:v.value for k,v in config.normalization_mapping.items()})
        report['delta_timestamps'] = delta
        stats = copy.deepcopy(dataset.meta.stats)
        quantiles = {}
        for key in ('observation.state','action'):
            s = stats[key]
            had = 'q01' in s and 'q99' in s
            if not had:
                values=np.asarray([row[key] for row in table],dtype=np.float64)
                s['q01'],s['q99']=np.quantile(values,[.01,.99],axis=0)
            lo,hi=np.asarray(s['q01']),np.asarray(s['q99'])
            assert lo.shape==hi.shape==(7,) and np.isfinite(lo).all() and np.isfinite(hi).all()
            spans=hi-lo
            assert (spans>0).all(),f'{key}: constant quantile dimension'
            quantiles[key]=dict(original_stats_present=had,q01=lo.tolist(),q99=hi.tolist(),
                                spans=spans.tolist(),all_positive=True,minimum_span=float(spans.min()))
        report['quantiles']=quantiles
        if not all(v['original_stats_present'] for v in quantiles.values()):
            (output/'quantile_override.json').write_text(json.dumps(quantiles,indent=2)+'\n')
        # These are the exact processor classes used by make_pi05_pre_post_processors.
        norm=NormalizerProcessorStep(features=features,norm_map=config.normalization_mapping,stats=stats,device='cpu')
        unnorm=UnnormalizerProcessorStep(features=features,norm_map=config.normalization_mapping,stats=stats,device='cpu')
        prepare_prompt=Pi05PrepareStateTokenizerProcessorStep(max_state_dim=32)

        # Instantiate only the modules named in the local PI05Pytorch constructor.
        # No PaliGemma construction, config mocking, or source file modification.
        width=modeling.get_gemma_config(config.action_expert_variant).width
        core=modeling.PI05Pytorch.__new__(modeling.PI05Pytorch)
        nn.Module.__init__(core)
        core.config=config
        core.gradient_checkpointing_enabled=False
        core.action_in_proj=nn.Linear(config.max_action_dim,width)
        core.action_out_proj=nn.Linear(width,config.max_action_dim)
        core.time_mlp_in=nn.Linear(width,width)
        core.time_mlp_out=nn.Linear(width,width)
        wrapper=modeling.PI05Policy.__new__(modeling.PI05Policy)
        nn.Module.__init__(wrapper)
        wrapper.config=config
        wrapper.model=core
        report['projection_parameters']=sum(p.numel() for p in core.parameters())
        report['projection_width']=width

        def vector_field(x,t):
            embeds,pad,attention,condition=core.embed_suffix(x,t)
            assert list(embeds.shape)==[2,50,width]
            assert list(pad.shape)==list(attention.shape)==[2,50]
            # Explicit minimal surrogate for the omitted transformer.
            return core.action_out_proj(embeds+condition[:,None,:])

        batches={}; image_sets={}
        for name,indices in selections.items():
            batch=next(iter(DataLoader(Subset(dataset,indices),batch_size=2,num_workers=0,shuffle=False)))
            assert list(batch['observation.state'].shape)==[2,7]
            assert list(batch['action'].shape)==[2,50,7]
            assert list(batch['action_is_pad'].shape)==[2,50]
            assert batch['action_is_pad'].dtype==torch.bool
            assert all(task==export['task'] for task in batch['task'])
            # Verify the actual offsets, endpoint repetition and mask for every step.
            expected_mask=[]
            for b,i in enumerate(indices):
                mask=[i+j>=len(table) for j in range(50)]
                expected_mask.append(mask)
                expected=np.asarray([table[min(i+j,len(table)-1)]['action'] for j in range(50)])
                np.testing.assert_allclose(batch['action'][b].numpy(),expected,atol=1e-6)
                assert batch['action_is_pad'][b].tolist()==mask
            raw_info={k:tensor_info(batch[k]) for k in [*cameras,'observation.state','action','action_is_pad']}
            entry=dict(indices=indices,stage=[export['enums']['task_stage'][int(scalar(table[i],'source.task_stage'))] for i in indices],
                       raw=raw_info,task=batch['task'],padding_counts=batch['action_is_pad'].sum(1).tolist(),
                       offset_and_padding_values_verified=True)
            transition={TK.OBSERVATION:{k:batch[k] for k in features if k!='action'},TK.ACTION:batch['action'],
                        TK.COMPLEMENTARY_DATA:{'task':batch['task']}}
            normalized=norm(transition)
            restored=unnorm(normalized)
            error=float((restored[TK.ACTION]-batch['action']).abs().max())
            assert error<1e-5
            entry['normalization_roundtrip_max_abs_error']=error
            prompted=prepare_prompt(copy.deepcopy(normalized))
            entry['state_prompt']=prompted[TK.COMPLEMENTARY_DATA]['task']
            processed, masks=wrapper._preprocess_images(batch)
            assert len(processed)==len(cameras)==3
            for image,mask in zip(processed,masks):
                assert list(image.shape)==[2,3,224,224] and image.dtype==torch.float32
                assert -1<=image.min() and image.max()<=1 and mask.all()
            entry['pi05_images']={k:tensor_info(v) for k,v in zip(cameras,processed)}
            image_sets[name]=(batch,processed)
            actions=wrapper.prepare_action({'action':normalized[TK.ACTION]})
            assert list(actions.shape)==[2,50,32] and torch.count_nonzero(actions[:,:,7:])==0
            noise=core.sample_noise(actions.shape,actions.device)
            times=core.sample_time(2,actions.device)
            x_t,model_time=modeling._build_flow_matching_inputs(actions,noise,times,None)
            u_t=noise-actions
            v_t=vector_field(x_t,model_time)
            losses=F.mse_loss(u_t,v_t,reduction='none')
            loss=losses[:,:,:7].mean()  # same actual-dimension slice as PI05Policy.forward
            valid=(~batch['action_is_pad']).unsqueeze(-1).expand(-1,-1,7)
            masked=losses[:,:,:7][valid].mean()
            assert torch.isfinite(loss) and torch.isfinite(masked)
            core.zero_grad(set_to_none=True)
            loss.backward()
            assert all(p.grad is not None and torch.isfinite(p.grad).all() for p in core.parameters())
            entry['flow_matching']=dict(loss=float(loss.detach()),valid_only_loss=float(masked.detach()),
                u_t_shape=list(u_t.shape),v_t_shape=list(v_t.shape),all_gradients_finite=True,
                surrogate='action_out_proj(action_in_proj(x_t) + time_mlp(sinusoidal(t)))',
                full_policy_forward=False)
            batches[name]=entry
            print('BATCH',name,json.dumps(entry,ensure_ascii=False),flush=True)
        report['batches']=batches
        assert batches['clamp_cut']['padding_counts']==[0,0]
        assert batches['episode_tail']['padding_counts']==[48,49]
        # One ten-step denoising decode with the same primitive as local PI05.
        with torch.no_grad():
            noise=core.sample_noise((2,50,32),'cpu')
            predicted=euler_integrate(vector_field,noise,config.num_inference_steps)[:,:,:7]
            physical=unnorm({TK.ACTION:predicted})[TK.ACTION]
            assert list(physical.shape)==[2,50,7] and torch.isfinite(physical).all()
        np.save(output/'projection_decoded_actions.npy',physical.numpy())
        report['inference']=dict(normalized_shape=list(predicted.shape),physical_shape=list(physical.shape),
            steps=config.num_inference_steps,all_finite=True,first_action=physical[0,0].tolist(),
            units=['rad']*6+['tool_absolute_normalized_0_open_1_closed'],
            physical=tensor_info(physical),random_untrained=True,not_a_binary_gripper_command=True,
            saved='projection_decoded_actions.npy')
        report['padding_loss_note']='Local PI05Policy.forward does not consume action_is_pad in its current reduction. Report native-style mean plus separately masked diagnostic loss; no library loss changes.'

        # Two batch representatives: original aspect-ratio previews + exact 224²
        # model inputs. 224 panels are displayed at native pixels, no enlargement.
        canvas=Image.new('RGB',(1332,760),'#f1f3f5');draw=ImageDraw.Draw(canvas)
        font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
        for section,(name,(batch,processed)) in enumerate(image_sets.items()):
            top=section*380
            draw.text((12,top+7),f'{name}: frame {selections[name][0]} | original (scaled for layout) / PI05 224 x 224',fill='black',font=font)
            for column,(key,img) in enumerate(zip(cameras,processed)):
                left=column*444+12
                raw_image=(batch[key][0].permute(1,2,0).clamp(0,1).numpy()*255).round().astype('uint8')
                original=Image.fromarray(raw_image);original.thumbnail((200,250))
                rgb=(((img[0]+1)/2).permute(1,2,0).clamp(0,1).numpy()*255).round().astype('uint8')
                draw.text((left,top+38),key.removeprefix('observation.images.'),fill='black',font=font)
                draw.text((left,top+60),f'raw {raw_image.shape[1]}x{raw_image.shape[0]}',fill='#555555',font=font)
                canvas.paste(original,(left,top+86))
                canvas.paste(Image.fromarray(rgb),(left+206,top+86))
                Image.fromarray(raw_image).save(output/f'{name}_{column}_original.png')
                Image.fromarray(rgb).save(output/f'{name}_{column}_224.png')
        canvas.save(output/'visual_comparison.png')
        report['preview']='visual_comparison.png'
        report['checks']=dict(chunk_shapes=True,delta_action_values=True,padding_mask=True,
            task_prompt=True,positive_quantile_spans=True,real_pi05_resize_pad=True,
            real_normalizer_roundtrip=True,projection_flow_loss=True,projection_backward=True,
            ten_step_projection_decode_7d=True)
        report['complete']=True
    finally:
        after=inventory(root)
        report['dataset_unchanged']=before==after
        report['dataset_sha256_before']=before
        report['dataset_sha256_after']=after
        report['elapsed_s']=time.monotonic()-started
        (output/'smoke_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if before!=after:
            raise RuntimeError('Original dataset changed during smoke test')
    write_readable_report(output, report)
    print('REPORT',output/'smoke_report.json',flush=True)


if __name__=='__main__':
    main()
