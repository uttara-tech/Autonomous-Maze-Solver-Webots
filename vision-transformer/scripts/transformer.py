import torch
import torch.nn as nn
import math
import pandas as pd
from torch.utils.data import Dataset
import torchvision.transforms as T
from PIL import Image

class ViTDataset(Dataset):
    
    def __init__(self,dataset_csv,imu_csv,image_dir,transform=None):
        self.data_df = pd.read_csv(dataset_csv)
        self.imu_df = pd.read_csv(imu_csv)
        self.image_dir = image_dir
        self.transform = transform 

    def __len__(self):
        return len(self.data_df)
    
        
    def __getitem__(self, index):
        img_path = str(self.data_df['img_path'].iloc[index])
        image = Image.open(img_path).convert('L')
        
        img_ts = str(self.data_df['timestamp'].iloc[index]).split('.')[0]
        img_ts = float(img_ts) / 1e9                                                # Convert to seconds

        img_start_ts = self.data_df['imu_start_ts'].iloc[index]
        img_end_ts = self.data_df['imu_end_ts'].iloc[index]

        mask = (self.imu_df['timestamp'] >= img_start_ts) & (self.imu_df['timestamp'] <= img_end_ts)

        imu_window = self.imu_df[mask]

        s = len(imu_window)

        if len(imu_window) > 0:                                                     # Selecting all IMU rows that fall within this window
            imu_sample = imu_window.iloc[:, 1:s].mean().values.astype('float32')    # Taking the average (mean) of Acc and Gyro over the window
        else:                                                                       # Fallback 
            mid_ts = (img_start_ts + img_end_ts) / 2
            closest_idx = (self.imu_df.iloc[:, index] - mid_ts).abs().idxmin()
            imu_sample = self.imu_df.iloc[closest_idx, index:s].values.astype('float32')

        target_pose = self.data_df.loc[index, ['pos_x', 'pos_y', 'pos_z']].values.astype('float32')
                
        if self.transform:
            image = self.transform(image)
            
        return image, torch.tensor(imu_sample), torch.tensor(target_pose)


    def forward(self, img, imu):
        visual_features = self.backbone(img)
        combined = torch.cat((visual_features, imu), dim=1)
        return self.regressor(combined)




# Current image size: (640, 480),
def extract_patches(image,patch_size=16):
    B, C, H, W = image.shape
    patches = image.unfold(2,patch_size,patch_size).unfold(3,patch_size,patch_size)
    patches = patches.contiguous().view(B,-1, patch_size,patch_size)       #Original (after contiguous): [1, 1, 16, 16, 16, 16], After View: [1, (1 * 16 * 16), 16, 16] --> [1,256,16,16]
    return patches  



class PatchEmbed(nn.Module):
    def __init__(self, img_size=(256,256),patch_size=16,embed_dim=256):
        super().__init__()
        self.img_size = img_size
        self.patch_size = patch_size
        self.num_patches = (self.img_size[0] // self.patch_size) * (self.img_size[1] // self.patch_size)
        patch_dim = self.patch_size * self.patch_size
        self.proj = nn.Linear(patch_dim,embed_dim)

    def forward(self,x):
        patches = extract_patches(x,self.patch_size)
        x = patches.flatten(2)              #Result: [1,256,256] --> merging H and W into 1D sequence
        x = self.proj(x)                    #Output: [1,256,out_dim] --> Learned projection of features
        return x
    
    def forward_o(self,x):
        B, C, H, W = x.shape
        x = x.unfold(2, self.patch_size, self.patch_size).unfold(3, self.patch_size, self.patch_size)
        x = x.contiguous().view(B, -1, self.patch_size*self.patch_size*C)
        x = self.proj(x)  # [B, 675, 256]
        return x
    

class MultiHeadAttention(nn.Module):
    def __init__(self, d_model, num_heads):
        super(MultiHeadAttention, self).__init__()
        #Ensure that the model dimesion (d_model) is divisible by num_heads
        assert d_model % num_heads == 0, 'd_model must be divisible by num_heads'

        #initialize dimesions
        self.d_model = d_model                          
        self.num_heads = num_heads                      #number of heads
        self.d_k = d_model // num_heads                 #number of dimensions per head

        #Linear layers for transforming inputs
        self.W_q = nn.Linear(d_model,d_model)
        self.W_k = nn.Linear(d_model,d_model)
        self.W_v = nn.Linear(d_model,d_model)
        self.W_o = nn.Linear(d_model,d_model)
    
    def scaled_dot_product_attention(self, Q, K, V, mask=None):
        #Calculate the attention scores
        attention_scores = torch.matmul(Q, K.transpose(-2,-1)) / math.sqrt(self.d_k)
        probs = torch.softmax(attention_scores, dim=-1)
        return torch.matmul(probs, V)
    
    def forward(self,x):
        B,N,C = x.shape
        Q = self.W_q(x).view(B, N, self.num_heads, self.d_k).transpose(1,2)
        K = self.W_k(x).view(B, N, self.num_heads, self.d_k).transpose(1,2)
        V = self.W_v(x).view(B, N, self.num_heads, self.d_k).transpose(1,2)

        attention = self.scaled_dot_product_attention(Q,K,V)
        attention = attention.transpose(1,2).contiguous().view(B,N,C)

        return self.W_o(attention)
    

class TransformerEncoderBlock(nn.Module):
    def __init__(self, d_model, num_heads, mlp_dim, dropout=0.1):
        super().__init__()
        self.attention = MultiHeadAttention(d_model, num_heads)
        self.mlp = nn.Sequential(
            nn.Linear(d_model,mlp_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(mlp_dim,d_model)
            )
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.dropout = nn.Dropout(dropout)
    
    def forward(self,x):
        attention = self.attention(x)
        x = self.norm1(x + self.dropout(attention))
        mlp = self.mlp(x)
        x = self.norm2(x + self.dropout(mlp))
        return x
        
    
class VisionTransformer(nn.Module):
    def __init__(self,
                 img_size=(256,256),
                 patch_size=16,
                 in_chans=1,
                 embed_dim=256,
                 depth=6,
                 n_heads=8,
                 mlp_dim=512,
                 dropout=0.1,
                 num_classes=None
                 ):
        super().__init__()
        self.patch_embed = PatchEmbed(img_size, patch_size, embed_dim)
        self.cls_token = nn.Parameter(torch.randn(1,1,embed_dim))
        self.position_embed = nn.Parameter(torch.randn(1,1 + self.patch_embed.num_patches, embed_dim))

        self.blocks = nn.ModuleList([
            TransformerEncoderBlock(embed_dim, n_heads, mlp_dim, dropout)
            for _ in range(depth)
        ])
        self.norm = nn.LayerNorm(embed_dim)
        self.head = nn.Linear(embed_dim, num_classes) if num_classes else nn.Identity()

    def forward(self,x):
        B = x.shape[0]
        x = self.patch_embed(x)
        cls = self.cls_token.expand(B, -1, -1)
        x = torch.cat([cls,x],dim = 1)
        x = x + self.position_embed
        
        for block in self.blocks:
            x = block(x)

        x = self.norm(x)
        cls_out = x[:,0]
        
        return self.head(cls_out)

model = VisionTransformer(num_classes=4)
img = torch.randn(1,1,256,256)
features = model(img)