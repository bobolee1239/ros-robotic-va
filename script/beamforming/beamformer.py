## *************************************
## FILE         : beamforming.py
## DESCRIPTION  : 
## *************************************
## AUTHOR       : Brian, LEE
##
##

import numpy.fft as fft
import numpy as np 

class Beamformer(object):
    """
    Virtual Base Class of each beamforming algorithm 
    """
    def __init__(self, radius=0.035, num_mics=6, resolution=1, fs=16000, nfft=1024):
        self.fs          = fs
        self.nfft        = int(nfft)
        self.radius      = radius
        self.num_mics    = num_mics
        self.direction   = 0      # Unit: degree
        self.resolution  = resolution

        self.speed_of_sound = 343.0  # Unit: meter/second
        self.angles = np.arange(0.0, 360.0, self.resolution)
        self.array_pos   = Beamformer.generate_uca_position(self.radius, 
                                                           self.num_mics)
        # TODO ...
        self.set_manifold_matrix_dict()

    def set_direction(self, buff):
        raise NotImplementedError

    def get_direction(self):
        return self.direction
   
    def sound_separation(self, buff):
        raise NotImplementedError
    
    @staticmethod
    def generate_uca_position(radius, num_mics, mic_theta=None):
        """
        base on respeaker microphone layout. 
        
        Arg:
        --------------------------
        radius  [float] : Unit: meter
        num_mics[int]   : # of mic in microphone array 
        """
        d_theta     = 2*np.pi / num_mics
        
        if mic_theta is None:
            mic_theta   = np.arange(d_theta, 2*np.pi, d_theta) 
        # mic_theta   = np.arange(d_theta/2, 2*np.pi, d_theta) 
        
        pos = radius * np.array([np.sin(mic_theta), np.cos(mic_theta)])

        return pos
    
    def create_manifold_matrix(self, freq, directions=None):
        angles = np.array(directions) if directions else self.angles
        return np.exp(1j*2.0*np.pi*freq/self.speed_of_sound *
                        np.array([np.sin(angles/180.0*np.pi), 
                                  np.cos(angles/180.0*np.pi)]).T.dot(self.array_pos)).T

    def set_manifold_matrix_dict(self):
        self.freq_axis = np.arange(0, self.nfft/2 + 1, 1) * self.fs / self.nfft
        self.manifold_matrix_dict = np.zeros((self.num_mics, len(self.angles), len(self.freq_axis)),  
                                                dtype='complex_')
        for i, f in enumerate(self.freq_axis):
           self.manifold_matrix_dict[:, :, i] = self.create_manifold_matrix(f) 

class CompressiveSensing(Beamformer):
    """
    """
    def __init__(self, cs_algo, radius=0.035, num_mics=6, resolution=5, fs=16000, nfft=1024):
        Beamformer.__init__(self, radius, num_mics, resolution, fs, nfft)
        self.cs_algo = cs_algo
    def set_direction(self, buff):
        """
        buff [np.ndarray] : shape = (#frames, #mics)
        """
        # take Fourier Transform 
        BUFF        = fft.rfft(buff, self.nfft, axis=0)
        
        scores = np.zeros(self.angles.shape)
        # iterating through each frequency 
        for i, f in enumerate(self.freq_axis):
            if f < 500:
                continue
            if f > 4000:
                break
            indices, _x = self.cs_algo(self.manifold_matrix_dict[:, :, i], 
                                       BUFF[i, :].T,
                                       0.8, 
                                       MAXITER=1)
            scores[indices] += np.abs(_x).flatten()  

        return self.angles[np.argmax(scores)]

    def sound_separation(self, buff, directions):
        BUFF = fft.rfft(buff, self.nfft, axis=0)

        # allocate memory for half of output 
        OUTPUT_H    = np.zeros((np.size(self.manifold_matrix_dict, 2), len(directions)), 
                               dtype = 'complex_')

        for i, f in enumerate(self.freq_axis):
            OUTPUT_H[i, :] = self.cs_algo(self.mainfold_matrix_dict[:, directions/self.resolution + 1, i], 
                                            BUFF[i, :].T, 
                                            0.1)
        # ifft 
        # return np.rifft(np.concatenate());
        raise NotImplementedError

    @staticmethod
    def OMP_SSL(A, b, sigma, MAXITER=100):
        """
        solving Ax = b and promoting sparsity

        A don't need to normalized for we are now in a plane wave model!
        
        Arg:
        --------------------------
        A [np.ndarray] : shape = (rows, cols), an underdetermined matrix
        b [np.ndarray] : shape = (rows, 1)

        Return:
        --------------------------
        x [np.ndarray] : shape = (cols, 1)
        """
        rows, cols = A.shape
        assert rows == b.shape[0], 'In OMP : Dim A and Dim b not matching!'
        # x = np.zeros((cols, 1), dtype='complex_')

        pos     = []
        # factors = []
        residue = b
        atoms   = None
        columns = []
        # 2-norm stopping criterion 
        criterion   = sigma * np.sqrt(rows + 2*np.sqrt(rows * np.log10(rows)));
        numIter     = 0
        while np.linalg.norm(residue, 2) > criterion:
            numIter += 1

            idx = np.argmax(np.abs(np.conj(A.T).dot(residue))) # find the max idx 
            pos.append(idx) 
            
            columns.append(A[:, idx])
            atoms = np.array(columns).T
            # update residue 
            # residue = residue - (A[:, idx].dot(residue)/rows * A[:, idx]).reshape(residue.shape)
            _x = np.linalg.pinv(atoms).dot(b)
            residue = b - atoms.dot(_x)

            if numIter >= MAXITER:
                break

        
        ## tmp = np.conj(_x) * (_x)
        ## max_var = np.conj(_x.T).dot(_x) 

        ## return np.array(pos)[(tmp > 0.3*max_var).flatten()]
        return np.array(pos), _x

    @staticmethod
    def OMP_SSS(A, b, sigma):
        """
        solving Ax = b and promoting sparsity

        A don't need to normalized for we are now in a plane wave model!
        
        Arg:
        --------------------------
        A       [np.ndarray] : shape = (rows, cols), an underdetermined matrix
        b       [np.ndarray] : shape = (rows, 1)
        sigma   [float]      : converge criterion

        Return:
        --------------------------
        x [np.ndarray] : shape = (cols, 1)
        """
        rows, cols = A.shape
        assert rows == b.shape[0], 'In OMP : Dim A and Dim b not matching!'
        x = np.zeros((cols, 1), dtype='complex_')

        pos     = []
        # factors = []
        residue = b
        atoms   = None
        columns = []
        # 2-norm stopping criterion 
        criterion   = sigma * np.sqrt(rows + 2*np.sqrt(rows * np.log10(rows)));
        numIter     = 0
        while np.linalg.norm(residue, 2) > criterion:
            numIter += 1

            idx = np.argmax(np.abs(np.conj(A.T).dot(residue))) # find the max idx 
            pos.append(idx) 
            
            columns.append(A[:, idx])
            atoms = np.array(columns).T
            # update residue 
            # residue = residue - (A[:, idx].dot(residue)/rows * A[:, idx]).reshape(residue.shape)
            _x = np.linalg.pinv(atoms).dot(b)
            residue = b - atoms.dot(_x)

            if numIter >= cols:
                break


        x[pos, 0] = _x.flatten()
        return x

    
class DAS(Beamformer):
    """
    """
    def __init__(self, eps=0.01, radius=0.035, num_mics=6, resolution=1, fs=16000, nfft=1024):
        Beamformer.__init__(self, radius, num_mics, resolution, fs, nfft)
        self.eps = eps

    def sound_separation(self, buf, directions):
        """
        Arg:
        ------------------------------------------------------
        buf        [np.ndarray] : shape = (#frames, #channels)
        directions [array]      :

        Returns:
        ------------------------------------------------------
        """
        buf = buf - np.mean(buf, axis=0)
        buf_len = np.size(buf, 0)
        # framing based DSP [rectangular window]
        nframes = int(buf_len / self.nfft) + 1
        buf = np.concatenate((buf, np.zeros((nframes*self.nfft-buf_len, np.size(buf, 1)))), 
                             axis=0)
        
        en_speech = []
        for n in range(nframes):
            BUFF = fft.rfft(buf[n*self.nfft:(n+1)*self.nfft], self.nfft, axis=0)
            # allocate memory for half of output, shape=(#nfft/2+1, #direction)
            OUTPUT_H    = np.zeros((np.size(self.manifold_matrix_dict, 2), len(directions)), 
                                   dtype = 'complex_')
    
            
            pos = [int(d/self.resolution) for d in directions]
    
            for i, f in enumerate(self.freq_axis):
                if i==0: continue
                m = self.manifold_matrix_dict[:, pos, i]
                tmp = np.array(BUFF[i, :], copy=True)
                OUTPUT_H[i, :] = np.linalg.inv(np.conj(m.T).dot(m) + self.eps*np.eye(len(directions))).dot(np.conj(m.T)).dot(tmp.reshape(self.num_mics, 1)).ravel()
    
            en_speech.append(fft.irfft(OUTPUT_H, self.nfft, axis=0))

        en_speech = np.array(en_speech).ravel()
        return en_speech

