/*
 * Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering,
 * New York University
 * Copyright (c) 2019 SIGNET Lab, Department of Information Engineering,
 * University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: 2025 Aleksei Fedorov <aleksei.fedorov@eit.lth.se>
// SPDX-FileCopyrightText: 2025 Nikita Lyamin <nikita.lyamin@volvocars.com>

#include "sivert-three-gpp-spectrum-propagation-loss-model.h"

#include "spectrum-signal-parameters.h"
#include "three-gpp-channel-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include <ns3/uinteger.h>

#include <map>
#include <fstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SivertThreeGppSpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(SivertThreeGppSpectrumPropagationLossModel);

SivertThreeGppSpectrumPropagationLossModel::SivertThreeGppSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

SivertThreeGppSpectrumPropagationLossModel::~SivertThreeGppSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

// Variables to track link IDs
// This is used to match links with complex channels passed from the Unity3D simulation.
std::unordered_map<std::pair<uint32_t, uint32_t>, int, SivertThreeGppSpectrumPropagationLossModel::PairHash> SivertThreeGppSpectrumPropagationLossModel::m_linkIdMap;
int SivertThreeGppSpectrumPropagationLossModel::m_nextLinkId = 0;

void SivertThreeGppSpectrumPropagationLossModel::DoInitialize()
{
  double init_gain = 0.0;
  std::complex<double> initComplexGain(0.0, 0.0);
  std::cout<< "Num of Sub-channels: " << m_number_of_RBs << std::endl;
  for (int i = 0; i < 10 ; ++i) {
      for (int j = 0; j < 10; ++j) {
          for (int k = 0; k < m_number_of_RBs; ++k)
            {
              m_Channel_from_Unity3D[i][j].push_back(init_gain);
              m_Channel_from_Unity3D_complex[i][j].push_back(initComplexGain);
            }
        }
    }
}

void
SivertThreeGppSpectrumPropagationLossModel::DoDispose()
{
    m_longTermMap.clear();
    m_channelModel->Dispose();
    m_channelModel = nullptr;
}

TypeId
SivertThreeGppSpectrumPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SivertThreeGppSpectrumPropagationLossModel")
            .SetParent<PhasedArraySpectrumPropagationLossModel>()
            .SetGroupName("Spectrum")
            .AddConstructor<SivertThreeGppSpectrumPropagationLossModel>()
            .AddAttribute(
                "ChannelModel",
                "The channel model. It needs to implement the MatrixBasedChannelModel interface",
                StringValue("ns3::SivertThreeGppChannelModel"),
                MakePointerAccessor(&SivertThreeGppSpectrumPropagationLossModel::SetChannelModel,
                                    &SivertThreeGppSpectrumPropagationLossModel::GetChannelModel),
                MakePointerChecker<MatrixBasedChannelModel>())
            .AddAttribute("NumRB",
                "Number of RB",
                UintegerValue (120),
                MakeUintegerAccessor(&SivertThreeGppSpectrumPropagationLossModel::m_number_of_RBs),
                MakeUintegerChecker<uint16_t> (1, 500))
            ;
    return tid;
}

void
SivertThreeGppSpectrumPropagationLossModel::SetChannelModel(Ptr<MatrixBasedChannelModel> channel)
{
    m_channelModel = channel;
}

Ptr<MatrixBasedChannelModel>
SivertThreeGppSpectrumPropagationLossModel::GetChannelModel() const
{
    return m_channelModel;
}

double
SivertThreeGppSpectrumPropagationLossModel::GetFrequency() const
{
    DoubleValue freq;
    m_channelModel->GetAttribute("Frequency", freq);
    return freq.Get();
}

void
SivertThreeGppSpectrumPropagationLossModel::SetChannelModelAttribute(const std::string& name,
                                                               const AttributeValue& value)
{
    m_channelModel->SetAttribute(name, value);
}

void
SivertThreeGppSpectrumPropagationLossModel::GetChannelModelAttribute(const std::string& name,
                                                               AttributeValue& value) const
{
    m_channelModel->GetAttribute(name, value);
}

Ptr<const MatrixBasedChannelModel::Complex3DVector>
SivertThreeGppSpectrumPropagationLossModel::CalcLongTerm(
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
    Ptr<const PhasedArrayModel> sAnt,
    Ptr<const PhasedArrayModel> uAnt) const
{
    NS_LOG_FUNCTION(this);

    const PhasedArrayModel::ComplexVector& sW = sAnt->GetBeamformingVectorRef();
    const PhasedArrayModel::ComplexVector& uW = uAnt->GetBeamformingVectorRef();
    size_t sAntNumElems = sW.GetSize();
    size_t uAntNumElems = uW.GetSize();
    NS_ASSERT(uAntNumElems == params->m_channel.GetNumRows());
    NS_ASSERT(sAntNumElems == params->m_channel.GetNumCols());
    // NS_LOG_DEBUG("CalcLongTerm with " << uW.GetSize() << " u antenna elements and " << sW.GetSize()
    //                                   << " s antenna elements, and with "
    //                                   << " s ports: " << sAnt->GetNumPorts()
    //                                   << " u ports: " << uAnt->GetNumPorts());
    NS_ASSERT_MSG((sAnt != nullptr) && (uAnt != nullptr), "Improper call to the method");
    size_t numClusters = params->m_channel.GetNumPages();
    // create and initialize the size of the longTerm 3D matrix
    Ptr<MatrixBasedChannelModel::Complex3DVector> longTerm =
        Create<MatrixBasedChannelModel::Complex3DVector>(uAnt->GetNumPorts(),
                                                         sAnt->GetNumPorts(),
                                                         numClusters);
    // Calculate long term uW * Husn * sW, the result is a matrix
    // with the dimensions #uPorts, #sPorts, #cluster
    NS_LOG_DEBUG("SIVERT Tx beam: " << sW << " Rx beam: " << uW);
    for (auto sPortIdx = 0; sPortIdx < sAnt->GetNumPorts(); sPortIdx++)
    {
        for (auto uPortIdx = 0; uPortIdx < uAnt->GetNumPorts(); uPortIdx++)
        {
            for (size_t cIndex = 0; cIndex < numClusters; cIndex++)
            {
                longTerm->Elem(uPortIdx, sPortIdx, cIndex) =
                    CalculateLongTermComponent(params, sAnt, uAnt, sPortIdx, uPortIdx, cIndex);
                // NS_LOG_DEBUG("uPortIdx: " << uPortIdx << ", sPortIdx: " << sPortIdx << ", cIndex: " << cIndex << ", value: " << longTerm->Elem(uPortIdx, sPortIdx, cIndex));
            }
        }
    }
    return longTerm;
}

std::complex<double>
SivertThreeGppSpectrumPropagationLossModel::CalculateLongTermComponent(
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
    Ptr<const PhasedArrayModel> sAnt,
    Ptr<const PhasedArrayModel> uAnt,
    uint16_t sPortIdx,
    uint16_t uPortIdx,
    uint16_t cIndex) const
{
    NS_LOG_FUNCTION(this);
    const PhasedArrayModel::ComplexVector& sW = sAnt->GetBeamformingVectorRef();
    const PhasedArrayModel::ComplexVector& uW = uAnt->GetBeamformingVectorRef();
    auto sPortElems = sAnt->GetNumElemsPerPort();
    auto uPortElems = uAnt->GetNumElemsPerPort();
    auto startS = sAnt->ArrayIndexFromPortIndex(sPortIdx, 0);
    auto startU = uAnt->ArrayIndexFromPortIndex(uPortIdx, 0);
    std::complex<double> txSum(0, 0);
    // limiting multiplication operations to the port location
    auto sIndex = startS;
    // The sub-array partition model is adopted for TXRU virtualization,
    // as described in Section 5.2.2 of 3GPP TR 36.897,
    // and so equal beam weights are used for all the ports.
    // Support of the full-connection model for TXRU virtualization would need extensions.
    const auto uElemsPerPort = uAnt->GetHElemsPerPort();
    const auto sElemsPerPort = sAnt->GetHElemsPerPort();
    for (size_t tIndex = 0; tIndex < sPortElems; tIndex++, sIndex++)
    {
        std::complex<double> rxSum(0, 0);
        auto uIndex = startU;
        for (size_t rIndex = 0; rIndex < uPortElems; rIndex++, uIndex++)
        {
            rxSum += uW[uIndex - startU] * params->m_channel(uIndex, sIndex, cIndex);
            auto testV = (rIndex % uElemsPerPort);
            auto ptInc = uElemsPerPort - 1;
            if (testV == ptInc)
            {
                auto incVal = uAnt->GetNumColumns() - uElemsPerPort;
                uIndex += incVal; // Increment by a factor to reach next column in a port
            }
        }

        txSum += sW[sIndex - startS] * rxSum;
        auto testV = (tIndex % sElemsPerPort);
        auto ptInc = sElemsPerPort - 1;
        if (testV == ptInc)
        {
            size_t incVal = sAnt->GetNumColumns() - sElemsPerPort;
            sIndex += incVal; // Increment by a factor to reach next column in a port
        }
    }
    return txSum;
}

Ptr<SpectrumSignalParameters>
SivertThreeGppSpectrumPropagationLossModel::CalcBeamformingGain(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MatrixBasedChannelModel::Complex3DVector> longTerm,
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams,
    const Vector& sSpeed,
    const ns3::Vector& uSpeed,
    uint8_t numTxPorts,
    uint8_t numRxPorts,
    bool isReverse) const

{
    NS_LOG_FUNCTION(this);
    Ptr<SpectrumSignalParameters> rxParams = params->Copy();
    size_t numCluster = channelMatrix->m_channel.GetNumPages();
    // compute the doppler term
    // NOTE the update of Doppler is simplified by only taking the center angle of
    // each cluster in to consideration.
    double slotTime = Simulator::Now().GetSeconds();
    double factor = 2 * M_PI * slotTime * GetFrequency() / 3e8;
    PhasedArrayModel::ComplexVector doppler(numCluster);

    // Make sure that all the structures that are passed to this function
    // are of the correct dimensions before using the operator [].
    NS_ASSERT(numCluster <= channelParams->m_alpha.size());
    NS_ASSERT(numCluster <= channelParams->m_D.size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX].size());
    NS_ASSERT(numCluster <= longTerm->GetNumPages());

    // check if channelParams structure is generated in direction s-to-u or u-to-s
    bool isSameDir = (channelParams->m_nodeIds == channelMatrix->m_nodeIds);

    // if channel params is generated in the same direction in which we
    // generate the channel matrix, angles and zenith of departure and arrival are ok,
    // just set them to corresponding variable that will be used for the generation
    // of channel matrix, otherwise we need to flip angles and zeniths of departure and arrival
    using DPV = std::vector<std::pair<double, double>>;
    using MBCM = MatrixBasedChannelModel;
    const auto& cachedAngleSincos = channelParams->m_cachedAngleSincos;
    const DPV& zoa = cachedAngleSincos[isSameDir ? MBCM::ZOA_INDEX : MBCM::ZOD_INDEX];
    const DPV& zod = cachedAngleSincos[isSameDir ? MBCM::ZOD_INDEX : MBCM::ZOA_INDEX];
    const DPV& aoa = cachedAngleSincos[isSameDir ? MBCM::AOA_INDEX : MBCM::AOD_INDEX];
    const DPV& aod = cachedAngleSincos[isSameDir ? MBCM::AOD_INDEX : MBCM::AOA_INDEX];

    for (size_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
        // Compute alpha and D as described in 3GPP TR 37.885 v15.3.0, Sec. 6.2.3
        // These terms account for an additional Doppler contribution due to the
        // presence of moving objects in the surrounding environment, such as in
        // vehicular scenarios.
        // This contribution is applied only to the delayed (reflected) paths and
        // must be properly configured by setting the value of
        // m_vScatt, which is defined as "maximum speed of the vehicle in the
        // layout".
        // By default, m_vScatt is set to 0, so there is no additional Doppler
        // contribution.

        double alpha = channelParams->m_alpha[cIndex];
        double D = channelParams->m_D[cIndex];

        // cluster angle angle[direction][n], where direction = 0(aoa), 1(zoa).
        double tempDoppler =
            factor *
            ((zoa[cIndex].first * aoa[cIndex].second * uSpeed.x +
              zoa[cIndex].first * aoa[cIndex].first * uSpeed.y + zoa[cIndex].second * uSpeed.z) +
             (zod[cIndex].first * aod[cIndex].second * sSpeed.x +
              zod[cIndex].first * aod[cIndex].first * sSpeed.y + zod[cIndex].second * sSpeed.z) +
             2 * alpha * D);
        doppler[cIndex] = std::complex<double>(cos(tempDoppler), sin(tempDoppler));
    }

    NS_ASSERT(numCluster <= doppler.GetSize());

    // set the channel matrix
    rxParams->spectrumChannelMatrix = GenSpectrumChannelMatrix(rxParams->psd,
                                                               longTerm,
                                                               channelMatrix,
                                                               channelParams,
                                                               doppler,
                                                               numTxPorts,
                                                               numRxPorts,
                                                               isReverse);

    // The precoding matrix is not set
    Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (rxParams->psd);
    Bands::const_iterator fit = rxPsd->ConstBandsBegin ();
    double rbWidth = fit->fh - fit->fl;
    NS_LOG_DEBUG("RB bandwidth: " << rbWidth);
    if (!rxParams->precodingMatrix)
    {
        // Update rxParams->Psd.
        // Compute RX PSD from the channel matrix
        auto vit = rxParams->psd->ValuesBegin(); // psd iterator
        size_t rbIdx = 0;
        NS_LOG_DEBUG("Operating in SISO mode, calculating RX PSD from the channel matrix");
        while (vit != rxParams->psd->ValuesEnd())
        {
            // Calculate PSD for the first antenna port (correct for SISO)
            *vit = std::norm(rxParams->spectrumChannelMatrix->Elem(0, 0, rbIdx));
            NS_LOG_DEBUG("SISO mode 3gpp spectrum RB " << rbIdx << " PSD value: " << *vit << " W/Hz " << "Power in dBm per 180kHz RB: "
                                                  << 10 * log10(*vit * rbWidth) + 30);
            vit++;
            rbIdx++;
        }
    }
    else
    {
        NS_ASSERT_MSG(rxParams->psd->GetValuesN() == rxParams->spectrumChannelMatrix->GetNumPages(),
                      "RX PSD and the spectrum channel matrix should have the same number of RBs ");
        // Calculate RX PSD from the spectrum channel matrix, H and
        // the precoding matrix, P as:
        // PSD = (H*P)^h * (H*P),
        // where the dimensions are:
        // H (rxPorts,txPorts,numRbs) x P (txPorts,txStreams, numRbs) =
        // HxP (rxPorts,txStreams, numRbs)
        
        // NS_LOG_DEBUG("Logging rxParams->precodingMatrix:");
        // for (uint32_t i = 0; i < rxParams->precodingMatrix->GetNumRows(); ++i)
        // {
        //     for (uint32_t j = 0; j < rxParams->precodingMatrix->GetNumCols(); ++j)
        //     {
        //         for (uint32_t k = 0; k < rxParams->precodingMatrix->GetNumPages(); ++k)
        //         {
        //             std::complex<double> val = rxParams->precodingMatrix->Elem(i, j, k);
        //             NS_LOG_DEBUG("precodingMatrix(" << i << ", " << j << ", " << k << ") = " << val);
        //         }
        //     }
        // }


        NS_LOG_DEBUG("MIMO precoding matrix in 3gpp spectrum model: "
                     << *rxParams->precodingMatrix);
        
        // Stream to file with simulation time
        std::ofstream outFile("mimo_psd_log_at_spectrum_channel.txt", std::ios_base::app); // append mode
        outFile <<  "Original PSD RB=50: " << (*rxParams->psd)[50];
                // << ", Original H matrix element (0,0,50): " << (*rxParams->spectrumChannelMatrix)(0, 0, 50);
                           

        MatrixBasedChannelModel::Complex3DVector hP =
            *rxParams->spectrumChannelMatrix * (*rxParams->precodingMatrix);
        NS_LOG_DEBUG("Size of hP: rows=" << hP.GetNumRows() << ", cols=" << hP.GetNumCols() << ", pages=" << hP.GetNumPages());

        // outFile << ", Precoding matrix element (0,0,50): " << (*rxParams->precodingMatrix)(0, 0, 50);

        // (HxP)^h dimensions are (txStreams, rxPorts, numRbs)
        MatrixBasedChannelModel::Complex3DVector hPHerm = hP.HermitianTranspose();
        NS_LOG_DEBUG("Size of (HxP).herm: txStreams=" << hPHerm.GetNumRows() << ", rxPorts=" << hPHerm.GetNumCols() << ", numRbs=" << hPHerm.GetNumPages());

        // outFile << ", HxP result (0,0,50): " << hP(0, 0, 50);

        // Finally, (HxP)^h x (HxP) = PSD (txStreams, txStreams, numRbs)
        MatrixBasedChannelModel::Complex3DVector psd = hPHerm * hP;

        // outFile << ", (HxP).Herm×(HxP) result (0,0,50): " << psd(0, 0, 50);    

        NS_LOG_DEBUG("Size of psd: txStreams=" << psd.GetNumRows() << ", txStreams=" << psd.GetNumCols() << ", numRbs=" << psd.GetNumPages());
        // Update rxParams->Psd
        for (uint32_t rbIdx = 0; rbIdx < rxParams->psd->GetValuesN(); ++rbIdx)
        {
            (*rxParams->psd)[rbIdx] = 0.0;


            for (size_t txStream = 0; txStream < psd.GetNumRows(); ++txStream)
            {
                (*rxParams->psd)[rbIdx] += std::real(psd(txStream, txStream, rbIdx));
                 NS_LOG_DEBUG("MIMO 3gpp spectrum RB " << rbIdx
                                                  << " PSD value for tx stream " << txStream
                                                  << ": " << (*rxParams->psd)[rbIdx] << " W/Hz " << "Power in dBm per 180kHz RB: "
                                                  << 10 * log10((*rxParams->psd)[rbIdx] * rbWidth) + 30);
                if (rbIdx==50)
                {
                    // // Stream to file with simulation time
                    // std::ofstream outFile("mimo_psd_log_at_spectrum_channel.txt", std::ios_base::app); // append mode
                    outFile << " Time: " <<  Simulator::Now().GetSeconds() << ", s "
                    << ", RB: " << rbIdx
                    << ", txStream: " << txStream
                    << ", PSD: " << (*rxParams->psd)[rbIdx]
                    << ", W/Hz Power in dBm per 180kHz RB: "
                    << 10 * log10((*rxParams->psd)[rbIdx] * rbWidth) + 30
                    << ", Channel coefficient (tx tx iRb): "
                    <<  GetComplexChannelElementFromUnity3D(txStream, txStream, rbIdx)
                    << " , rxParams->SpectrumChannelMatrix: "
                    << (*rxParams->spectrumChannelMatrix)(txStream, txStream, rbIdx)
                    << std::endl; 
                }
                
            }
        }
        
    }
    return rxParams;
}

Ptr<MatrixBasedChannelModel::Complex3DVector>
SivertThreeGppSpectrumPropagationLossModel::GenSpectrumChannelMatrix(
    Ptr<SpectrumValue> inPsd,
    Ptr<const MatrixBasedChannelModel::Complex3DVector> longTerm,
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams,
    PhasedArrayModel::ComplexVector doppler,
    uint8_t numTxPorts,
    uint8_t numRxPorts,
    bool isReverse) const
{
    size_t numCluster = channelMatrix->m_channel.GetNumPages();
    auto numRb = inPsd->GetValuesN();

    auto directionalLongTerm = isReverse ? longTerm->Transpose() : (*longTerm);

    Ptr<MatrixBasedChannelModel::Complex3DVector> chanSpct =
        Create<MatrixBasedChannelModel::Complex3DVector>(numRxPorts, numTxPorts, (uint16_t)numRb);

    // Precompute the delay until numRb, numCluster or RB width changes
    // Whenever the channelParams is updated, the number of numRbs, numClusters
    // and RB width (12*SCS) are reset, ensuring these values are updated too
    double rbWidth = inPsd->ConstBandsBegin()->fh - inPsd->ConstBandsBegin()->fl;

    if (channelParams->m_cachedDelaySincos.GetNumRows() != numRb ||
        channelParams->m_cachedDelaySincos.GetNumCols() != numCluster ||
        channelParams->m_cachedRbWidth != rbWidth)
    {
        channelParams->m_cachedRbWidth = rbWidth;
        channelParams->m_cachedDelaySincos = ComplexMatrixArray(numRb, numCluster);
        auto sbit = inPsd->ConstBandsBegin(); // band iterator
        for (unsigned i = 0; i < numRb; i++)
        {
            double fsb = (*sbit).fc; // center frequency of the sub-band
            for (std::size_t cIndex = 0; cIndex < numCluster; cIndex++)
            {
                double delay = -2 * M_PI * fsb * (channelParams->m_delay[cIndex]);
                channelParams->m_cachedDelaySincos(i, cIndex) =
                    std::complex<double>(cos(delay), sin(delay));
            }
            sbit++;
        }
    }

    // Compute the product between the doppler and the delay sincos
    auto delaySincosCopy = channelParams->m_cachedDelaySincos;
    for (size_t iRb = 0; iRb < inPsd->GetValuesN(); iRb++)
    {
        for (std::size_t cIndex = 0; cIndex < numCluster; cIndex++)
        {
            delaySincosCopy(iRb, cIndex) *= doppler[cIndex];
        }
    }

    // If "params" (ChannelMatrix) and longTerm were computed for the reverse direction (e.g. this
    // is a DL transmission but params and longTerm were last updated during UL), then the elements
    // in longTerm start from different offsets.

    auto vit = inPsd->ValuesBegin(); // psd iterator
    size_t iRb = 0;
    
    // SIVERT: Mapping the node pair to a link ID for correct channel mattrix generation
    // This is used to match links with complex channels passed from the Unity3D simulation.
    auto nodePair = channelParams->m_nodeIds;
    int linkId = -1;
    auto it = m_linkIdMap.find(nodePair);
    if (it != m_linkIdMap.end()) {
        linkId = it->second;
    }
    NS_LOG_DEBUG("[GenSpectrumChannelMatrix] nodePair (" << nodePair.first << ", " << nodePair.second << ") maps to LinkID " << linkId);

    int numTxRxPorts = numTxPorts + numRxPorts;

    NS_LOG_DEBUG("Generating spectrum channel matrix for link ID: " << linkId
                 << ", numTxPorts: " << numTxPorts
                 << ", numRxPorts: " << numRxPorts
                 << ", numRb: " << numRb);
    while (vit != inPsd->ValuesEnd())
    {
        if ((*vit) != 0.00)
        {
            auto sqrtVit = sqrt(*vit);
            for (auto rxPortIdx = 0; rxPortIdx < numRxPorts; rxPortIdx++)
            {
                for (auto txPortIdx = 0; txPortIdx < numTxPorts; txPortIdx++)
                {
                    std::complex<double> subsbandGain(0.0, 0.0);
                    for (size_t cIndex = 0; cIndex < numCluster; cIndex++)
                    {
                        // subsbandGain += directionalLongTerm(rxPortIdx, txPortIdx, cIndex) *
                        //                 delaySincosCopy(iRb, cIndex); // SIVERT: Remove the delay component - accounted for in the H(u,s,rB) Unity
                        subsbandGain += directionalLongTerm(rxPortIdx, txPortIdx, cIndex); //* sivert_LoS_bench;

                    }

                    auto unityCoeff = GetComplexChannelElementFromUnity3D(rxPortIdx, txPortIdx, iRb); // SIVERT: Get the complex channel coefficient from Unity3D simulation
                    chanSpct->Elem(rxPortIdx, txPortIdx, iRb) = sqrtVit * unityCoeff;
                    
                    // NS_LOG_DEBUG("chanSpct(" << rxPortIdx << ", " << txPortIdx << ", " << iRb << ") = "
                    //           << chanSpct->Elem(rxPortIdx, txPortIdx, iRb)); 
                }
            }
        }
        vit++;
        iRb++;
    }
    return chanSpct;
}

Ptr<const MatrixBasedChannelModel::Complex3DVector>
SivertThreeGppSpectrumPropagationLossModel::GetLongTerm(
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    Ptr<const MatrixBasedChannelModel::Complex3DVector>
        longTerm; // vector containing the long term component for each cluster

    // check if the channel matrix was generated considering a as the s-node and
    // b as the u-node or vice-versa
    auto isReverse =
        channelMatrix->IsReverse(aPhasedArrayModel->GetId(), bPhasedArrayModel->GetId());
    auto sAntenna = isReverse ? bPhasedArrayModel : aPhasedArrayModel;
    auto uAntenna = isReverse ? aPhasedArrayModel : bPhasedArrayModel;

    PhasedArrayModel::ComplexVector sW;
    PhasedArrayModel::ComplexVector uW;
    if (!isReverse)
    {
        sW = aPhasedArrayModel->GetBeamformingVector();
        uW = bPhasedArrayModel->GetBeamformingVector();
    }
    else
    {
        sW = bPhasedArrayModel->GetBeamformingVector();
        uW = aPhasedArrayModel->GetBeamformingVector();
    }

    bool update = false;   // indicates whether the long term has to be updated
    bool notFound = false; // indicates if the long term has not been computed yet

    // compute the long term key, the key is unique for each tx-rx pair
    uint64_t longTermId =
        MatrixBasedChannelModel::GetKey(aPhasedArrayModel->GetId(), bPhasedArrayModel->GetId());

    // look for the long term in the map and check if it is valid
    if (m_longTermMap.find(longTermId) != m_longTermMap.end())
    {
        NS_LOG_DEBUG("found the long term component in the map");
        longTerm = m_longTermMap[longTermId]->m_longTerm;

        // check if the channel matrix has been updated
        // or the s beam has been changed
        // or the u beam has been changed
        update = (m_longTermMap[longTermId]->m_channel->m_generatedTime !=
                      channelMatrix->m_generatedTime ||
                  m_longTermMap[longTermId]->m_sW != sW || m_longTermMap[longTermId]->m_uW != uW);
    }
    else
    {
        NS_LOG_DEBUG("long term component NOT found");
        notFound = true;
    }

    if (update || notFound)
    {
        NS_LOG_DEBUG("compute the long term");
        // compute the long term component
        longTerm = CalcLongTerm(channelMatrix, sAntenna, uAntenna);
        Ptr<LongTerm> longTermItem = Create<LongTerm>();
        longTermItem->m_longTerm = longTerm;
        longTermItem->m_channel = channelMatrix;
        longTermItem->m_sW = std::move(sW);
        longTermItem->m_uW = std::move(uW);
        // store the long term to reduce computation load
        // only the small scale fading needs to be updated if the large scale parameters and antenna
        // weights remain unchanged.
        m_longTermMap[longTermId] = longTermItem;
    }

    return longTerm;
}

Ptr<SpectrumSignalParameters>
SivertThreeGppSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> spectrumSignalParams,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    NS_LOG_FUNCTION(this << spectrumSignalParams << a << b << aPhasedArrayModel
                         << bPhasedArrayModel);

    uint32_t aId = a->GetObject<Node>()->GetId(); // id of the node a
    uint32_t bId = b->GetObject<Node>()->GetId(); // id of the node b
    NS_ASSERT_MSG(aPhasedArrayModel, "Antenna not found for node " << aId);
    NS_LOG_DEBUG("a node " << aId << " antenna " << aPhasedArrayModel);
    NS_ASSERT_MSG(bPhasedArrayModel, "Antenna not found for node " << bId);
    NS_LOG_DEBUG("b node " << bId << " antenna " << bPhasedArrayModel);

    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix =
        m_channelModel->GetChannel(a, b, aPhasedArrayModel, bPhasedArrayModel);
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams =
        m_channelModel->GetParams(a, b);

    // Create a mapping between the nodes IDs and the link ID to track the link IDs
    // and avoid reassigning them for the same node pair.
    // This is used to match links with complex channels passed form the Unity3D simulation.   
    auto nodePair = channelParams->m_nodeIds;
    if (m_linkIdMap.find(nodePair) == m_linkIdMap.end()) {
        m_linkIdMap[nodePair] = m_nextLinkId++;
        NS_LOG_DEBUG("Assigned LinkID " << m_linkIdMap[nodePair] << " to node pair (" << nodePair.first << ", " << nodePair.second << ")");
    }    

    // retrieve the long term component
    Ptr<const MatrixBasedChannelModel::Complex3DVector> longTerm =
        GetLongTerm(channelMatrix, aPhasedArrayModel, bPhasedArrayModel);

    auto isReverse =
        channelMatrix->IsReverse(aPhasedArrayModel->GetId(), bPhasedArrayModel->GetId());

    // apply the beamforming gain
    return CalcBeamformingGain(spectrumSignalParams,
                               longTerm,
                               channelMatrix,
                               channelParams,
                               a->GetVelocity(),
                               b->GetVelocity(),
                               aPhasedArrayModel->GetNumPorts(),
                               bPhasedArrayModel->GetNumPorts(),
                               isReverse);
}

int64_t
SivertThreeGppSpectrumPropagationLossModel::DoAssignStreams(int64_t stream)
{
    return 0;
}

void
SivertThreeGppSpectrumPropagationLossModel::SetChannelFromUnity3D(int TxID, int RxID, std::vector<double> gainVector)
{

  m_Channel_from_Unity3D[TxID][RxID] = gainVector;
//   m_Channel_from_Unity3D[RxID][TxID] = gainVector;

}

void
SivertThreeGppSpectrumPropagationLossModel::SetComplexChannelFromUnity3D(int TxID, int RxID, std::vector<std::complex<double>> complexChannel)
{

  m_Channel_from_Unity3D_complex[TxID][RxID] = complexChannel;
//   m_Channel_from_Unity3D_complex[RxID][TxID] = complexChannel;
//   m_Channel_from_Unity3D[RxID][TxID] = gainVector;

}

std::vector<double> SivertThreeGppSpectrumPropagationLossModel::GetChannelVectorFromUnity3D(int TxID, int RxID){
  return m_Channel_from_Unity3D[TxID][RxID];
}

std::vector<std::complex<double>> SivertThreeGppSpectrumPropagationLossModel::GetComplexChannelVectorFromUnity3D(int TxID, int RxID) const
{
  return m_Channel_from_Unity3D_complex[TxID][RxID];
}

std::complex<double> SivertThreeGppSpectrumPropagationLossModel::GetComplexChannelElementFromUnity3D(int TxID, int RxID, int iRb) const
{

  return m_Channel_from_Unity3D_complex[TxID][RxID][iRb]; // Return the first element as an example

}


}// namespace ns3
